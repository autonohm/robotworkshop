#include "MotorControllerCAN.h"
#include <iostream>
#include <cmath>
#include <string.h>
#include <iomanip>
#include <unistd.h>

#define GROUPID    (0x01 << 9)
#define SYSTEMID   (0x4  << 6)
#define COMPINPUT  (0x0  << 5)
#define COMPOUTPUT (0x1  << 5)

// Common parameters
#define CMD_ENABLE          0x01
#define CMD_DISABLE         0x02
#define CMD_SETTIMEOUT      0x03

// Operating commands
#define CMD_SETPWM          0x10
#define CMD_SETRPM          0x11

// Closed/Open loop controller parameters
#define CMD_CTL_KP          0x20
#define CMD_CTL_KI          0x21
#define CMD_CTL_KD          0x22
#define CMD_CTL_ANTIWINDUP  0x23
#define CMD_CTL_INPUTFILTER 0x24

// Platform parameters
#define CMD_GEARRATIO       0x30
#define CMD_GEARRATIO2      0x31
#define CMD_TICKSPERREV     0x32
#define CMD_TICKSPERREV2    0x33

MotorControllerCAN::MotorControllerCAN(SocketCAN* can, unsigned short channel)
{
  _can   = can;
  _cf.can_id = GROUPID | SYSTEMID | COMPINPUT | channel;

  canid_t canidOutput = GROUPID | SYSTEMID | COMPOUTPUT | channel;

  setCANId(canidOutput);
  can->registerObserver(this);

  _rpm[0] = 0.f;
  _rpm[1] = 0.f;

  _idSyncSend    = 0;
  _idSyncReceive = 0;
}

MotorControllerCAN::~MotorControllerCAN()
{
  stop();
}

bool MotorControllerCAN::enable()
{
  _cf.can_dlc = 1;
  _cf.data[0] = CMD_ENABLE;
  return _can->send(&_cf);
}

bool MotorControllerCAN::disable()
{
  _cf.can_dlc = 1;
  _cf.data[0] = CMD_DISABLE;
  return _can->send(&_cf);
}

bool MotorControllerCAN::setTimeout(unsigned short timeoutInMillis)
{
  _cf.can_dlc = 3;
  _cf.data[0] = CMD_SETTIMEOUT;
  _cf.data[1] = (timeoutInMillis >> 8) & 0xFF;
  _cf.data[2] = timeoutInMillis & 0xFF;
  return _can->send(&_cf);
}

bool MotorControllerCAN::setGearRatio(float gearRatio[2])
{
  bool retval = true;

  _cf.can_dlc = 5;
  _cf.data[0] = CMD_GEARRATIO;
  unsigned int* gr = (unsigned int*)&gearRatio[0];
  _cf.data[1] = (*gr >> 24) & 0xFF;
  _cf.data[2] = (*gr >> 16) & 0xFF;
  _cf.data[3] = (*gr >> 8)  & 0xFF;
  _cf.data[4] = *gr         & 0xFF;
  retval = _can->send(&_cf);

  _cf.data[0] = CMD_GEARRATIO2;
  gr = (unsigned int*)&gearRatio[1];
  _cf.data[1] = (*gr >> 24) & 0xFF;
  _cf.data[2] = (*gr >> 16) & 0xFF;
  _cf.data[3] = (*gr >> 8)  & 0xFF;
  _cf.data[4] = *gr         & 0xFF;
  retval &= _can->send(&_cf);

  return retval;
}

bool MotorControllerCAN::setEncoderTicksPerRev(float encoderTicksPerRev[2])
{
  bool retval = true;

  _cf.can_dlc = 5;
  _cf.data[0] = CMD_TICKSPERREV;
  unsigned int* et = (unsigned int*)&encoderTicksPerRev[0];
  _cf.data[1] = (*et >> 24) & 0xFF;
  _cf.data[2] = (*et >> 16) & 0xFF;
  _cf.data[3] = (*et >> 8)  & 0xFF;
  _cf.data[4] = *et         & 0xFF;
  retval = _can->send(&_cf);

  et = (unsigned int*)&encoderTicksPerRev[1];
  _cf.data[0] = CMD_TICKSPERREV2;
  _cf.data[1] = (*et >> 24) & 0xFF;
  _cf.data[2] = (*et >> 16) & 0xFF;
  _cf.data[3] = (*et >> 8)  & 0xFF;
  _cf.data[4] = *et         & 0xFF;
  retval &= _can->send(&_cf);

  return retval;
}

bool MotorControllerCAN::setPWM(int pwm[2])
{
  _cf.can_dlc = 3;

  int vel1 = (((int)pwm[0]) * 127) / 100;
  int vel2 = (((int)pwm[1]) * 127) / 100;
  _cf.data[0] = CMD_SETPWM;
  _cf.data[1] = (char)(vel1 + 0x7F);
  _cf.data[2] = (char)(vel2 + 0x7F);

  _idSyncSend++;
  return _can->send(&_cf);;
}

bool MotorControllerCAN::setRPM(float rpm[2])
{
  std::cout << "setRPM method not implemented yet" << std::endl;
  _idSyncSend++;
  return false;
}

void MotorControllerCAN::getRPM(float rpm[2])
{
  rpm[0] = _rpm[0];
  rpm[1] = _rpm[1];
}

void MotorControllerCAN::notify(struct can_frame* frame)
{
  if(frame->can_dlc==5)
  {
    _rpm[0] = ((float)((short)(frame->data[1] | (frame->data[2] << 8))))/100.f;
    _rpm[1] = ((float)((short)(frame->data[3] | (frame->data[4] << 8))))/100.f;
    _idSyncReceive = _idSyncSend;
  }
}

bool MotorControllerCAN::waitForSync(unsigned int timeoutInMillis)
{
  unsigned int cnt=0;
  bool synchronized = (_idSyncReceive==_idSyncSend);
  while(!synchronized && ((cnt++)<timeoutInMillis || timeoutInMillis==0))
  {
    usleep(1000);
    synchronized = (_idSyncReceive==_idSyncSend);
  }
  return synchronized;
}

void MotorControllerCAN::stop()
{
  _cf.can_dlc = 3;

  _cf.data[0] = CMD_SETPWM;
  _cf.data[1] = 0x7F;
  _cf.data[2] = 0x7F;
  _can->send(&_cf);
}
