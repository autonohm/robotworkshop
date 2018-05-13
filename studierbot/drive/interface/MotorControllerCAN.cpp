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
  _cf.data[0] = 0x01;
  return _can->send(&_cf);
}

bool MotorControllerCAN::setGearRatio(float gearRatio[2])
{
  bool retval = true;

  _cf.can_dlc = 5;
  _cf.data[0] = 0x30;
  unsigned int* gr = (unsigned int*)&gearRatio[0];
  _cf.data[1] = (*gr >> 24) & 0xFF;
  _cf.data[2] = (*gr >> 16) & 0xFF;
  _cf.data[3] = (*gr >> 8)  & 0xFF;
  _cf.data[4] = *gr         & 0xFF;
  retval = _can->send(&_cf);

  _cf.data[0] = 0x31;
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
  _cf.data[0] = 0x32;
  unsigned int* et = (unsigned int*)&encoderTicksPerRev[0];
  _cf.data[1] = (*et >> 24) & 0xFF;
  _cf.data[2] = (*et >> 16) & 0xFF;
  _cf.data[3] = (*et >> 8)  & 0xFF;
  _cf.data[4] = *et         & 0xFF;
  retval = _can->send(&_cf);

  et = (unsigned int*)&encoderTicksPerRev[1];
  _cf.data[0] = 0x33;
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
  _cf.data[0] = 0x10;
  _cf.data[1] = (char)(vel1 + 0x7F);
  _cf.data[2] = (char)(vel2 + 0x7F);

  _idSyncSend++;
  return _can->send(&_cf);;
}

bool MotorControllerCAN::setRPM(float rpmIn[2])
{
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

bool MotorControllerCAN::waitForSync()
{
  while(_idSyncReceive!=_idSyncSend)
  {
    usleep(10);
  }
  return true;
}

void MotorControllerCAN::stop()
{
  _cf.can_dlc = 3;

  _cf.data[0] = 0x10;
  _cf.data[1] = 0x7F;
  _cf.data[2] = 0x7F;
  _can->send(&_cf);
}
