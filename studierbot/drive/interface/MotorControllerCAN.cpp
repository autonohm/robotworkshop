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
#define CMD_SETPWMMAX       0x04
#define CMD_SENDRPM         0x05
#define CMD_SENDPOS         0x06

// Operating commands
#define CMD_SETPWM          0x10
#define CMD_SETRPM          0x11
#define CMD_FREQ_SCALE      0x12
#define CMD_EXT_SYN         0x13

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

// Standard responses
#define RESPONSE_RPM        0xA0
#define RESPONSE_POS        0xA1

// Error responses
#define ERR_ENCA_NOSIGNAL   0xE0
#define ERR_ENCB_NOSIGNAL   0xE1

MotorControllerCAN::MotorControllerCAN(SocketCAN* can, unsigned short channel)
{
  _channel   = channel;
  _can       = can;
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

bool MotorControllerCAN::broadcastExternalSync()
{
  _cf.can_id = GROUPID | SYSTEMID | COMPINPUT | 0x1F;
  _cf.can_dlc = 1;
  _cf.data[0] = CMD_EXT_SYN;
  bool retval = _can->send(&_cf);;
  _cf.can_id = GROUPID | SYSTEMID | COMPINPUT | _channel;
  return retval;
}

bool MotorControllerCAN::configureResponse(enum CanResponse mode)
{
  _cf.can_dlc = 1;
  if(mode==CAN_RESPONSE_RPM)
    _cf.data[0] = CMD_SENDRPM;
  else
    _cf.data[0] = CMD_SENDPOS;
  return _can->send(&_cf);
}

unsigned short MotorControllerCAN::getChannel()
{
  return _channel;
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
  bool retval = sendFloat(CMD_GEARRATIO, gearRatio[0]);
  retval &= sendFloat(CMD_GEARRATIO2, gearRatio[1]);

  return retval;
}

bool MotorControllerCAN::setEncoderTicksPerRev(float encoderTicksPerRev[2])
{
  bool retval = sendFloat(CMD_TICKSPERREV, encoderTicksPerRev[0]);
  retval &= sendFloat(CMD_TICKSPERREV2, encoderTicksPerRev[1]);

  return retval;
}

bool MotorControllerCAN::setFrequencyScale(unsigned short scale)
{
  bool retval = false;

  if(scale>0 && scale<=100)
  {
    _cf.can_dlc = 3;
    _cf.data[0] = CMD_FREQ_SCALE;
    _cf.data[1] = (scale >> 8) & 0xFF;
    _cf.data[2] = scale & 0xFF;
    retval = _can->send(&_cf);
  }
  return retval;
}

bool MotorControllerCAN::setMaxPulseWidth(unsigned char pulse)
{
  bool retval = false;

  if(pulse<=127)
  {
    _cf.can_dlc = 2;
    _cf.data[0] = CMD_SETPWMMAX;
    _cf.data[1] = pulse;
    retval = _can->send(&_cf);
  }
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
  return _can->send(&_cf);
}

bool MotorControllerCAN::setRPM(float rpm[2])
{
  _cf.can_dlc = 5;

  int vel1 = (int)(rpm[0]*10.f);
  int vel2 = (int)(rpm[1]*10.f);
  _cf.data[0] = CMD_SETRPM;
  _cf.data[1] = (char)(vel1 >> 8) & 0xFF;
  _cf.data[2] = (char)(vel1)      & 0xFF;
  _cf.data[3] = (char)(vel2 >> 8) & 0xFF;
  _cf.data[4] = (char)(vel2)      & 0xFF;

  _idSyncSend++;

  return _can->send(&_cf);
}

void MotorControllerCAN::getRPM(float rpm[2])
{
  rpm[0] = _rpm[0];
  rpm[1] = _rpm[1];
}

void MotorControllerCAN::getPos(short pos[2])
{
  pos[0] = _pos[0];
  pos[1] = _pos[1];
}

bool MotorControllerCAN::setKp(float kp)
{
  return sendFloat(CMD_CTL_KP, kp);
}

bool MotorControllerCAN::setKi(float ki)
{
  return sendFloat(CMD_CTL_KI, ki);
}

bool MotorControllerCAN::setKd(float kd)
{
  return sendFloat(CMD_CTL_KD, kd);
}

bool MotorControllerCAN::setInputWeight(float weight)
{
  return sendFloat(CMD_CTL_INPUTFILTER, weight);
}

void MotorControllerCAN::notify(struct can_frame* frame)
{
  if(frame->can_dlc==5)
  {
    if(frame->data[0] == RESPONSE_RPM)
    {
      short val1 = (frame->data[1] | (frame->data[2] << 8));
      short val2 = (frame->data[3] | (frame->data[4] << 8));
      _rpm[0] = ((float)val1)/10.f;
      _rpm[1] = ((float)val2)/10.f;
      _pos[0] = 0.f;
      _pos[1] = 0.f;
    }
    else if(frame->data[0] == RESPONSE_POS)
    {
      _rpm[0] = 0.f;
      _rpm[1] = 0.f;
      _pos[0] = (frame->data[1] | (frame->data[2] << 8));
      _pos[1] = (frame->data[3] | (frame->data[4] << 8));
    }
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

bool MotorControllerCAN::sendFloat(int cmd, float f)
{
  _cf.can_dlc = 5;

  _cf.data[0] = cmd;
  int* ival = (int*)&f;
  _cf.data[1] = (*ival & 0xFF000000) >> 24;
  _cf.data[2] = (*ival & 0x00FF0000) >> 16;
  _cf.data[3] = (*ival & 0x0000FF00) >> 8;
  _cf.data[4] = (*ival & 0x000000FF);

  _idSyncSend++;

  return _can->send(&_cf);
}
