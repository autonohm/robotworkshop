#include "AddonShieldCAN.h"
#include <iostream>
#include <cmath>
#include <string.h>
#include <iomanip>
#include <unistd.h>

#define GROUPID    (0x01 << 9)
#define SYSTEMID   (0x5  << 6)
#define COMPINPUT  (0x0  << 5)
#define COMPOUTPUT (0x1  << 5)

// Common parameters
#define CMD_ENABLE          0x01
#define CMD_DISABLE         0x02
#define CMD_SET_FREQUENCY   0x03
#define CMD_SET_PULSEWIDTH  0x04
#define CMD_SET_THRESHOLD   0x05
#define CMD_CH3_SET_12V     0x06
#define CMD_CH3_SET_5V      0x07
#define CMD_CH4_SET_19V     0x08
#define CMD_CH4_SET_5V      0x09

// Standard responses
#define RESPONSE_VOLTAGE    0xA0

AddonShieldCAN::AddonShieldCAN(SocketCAN* can, bool verbosity)
{
  _can       = can;
  _cf.can_id = GROUPID | SYSTEMID | COMPINPUT | 0x0;

  canid_t canidOutput = GROUPID | SYSTEMID | COMPOUTPUT | 0x0;

  setCANId(canidOutput);
  can->registerObserver(this);

  _idSyncSend    = 0;
  _idSyncReceive = 0;

  _voltage       = 0.f;

  bool retval = true;
  if(!setPWMFrequency(2, 100))
  {
    std::cout << "# Setting frequency scaling parameter failed for addon shield" << std::endl;
    retval = false;
  }
  usleep(1000);
  if(!enable(2))
  {
    std::cout << "# Enabling channel 2 failed for addon shield " << std::endl;
    retval = false;
  }

  usleep(25000);

  if(!retval)
  {
    std::cout << "# ERROR initializing addon shield" << std::endl;
    std::cout << "-----------------------------------------------";
  }
}

AddonShieldCAN::~AddonShieldCAN()
{

}

bool AddonShieldCAN::enable(unsigned char channel)
{
  if(channel>1 && channel<5)
  {
    _cf.can_dlc = 2;
    _cf.data[0] = CMD_ENABLE;
    _cf.data[1] = channel;
    _idSyncSend++;
    return _can->send(&_cf);
  }
  else
    return false;
}

bool AddonShieldCAN::disable(unsigned char channel)
{
  if(channel>1 && channel<5)
  {
    _cf.can_dlc = 2;
    _cf.data[0] = CMD_DISABLE;
    _cf.data[1] = channel;
    _idSyncSend++;
    return _can->send(&_cf);
  }
  else
    return false;
}

float AddonShieldCAN::getVoltage()
{
  return _voltage;
}

bool AddonShieldCAN::setThreshold(unsigned char channel, float threshold)
{
  _cf.can_dlc = 6;
  _cf.data[0] = CMD_SET_THRESHOLD;
  int* iThresh = (int*)&threshold;
  _cf.data[1] = channel;
  _cf.data[2] = (*iThresh & 0xFF000000) >> 24;
  _cf.data[3] = (*iThresh & 0x00FF0000) >> 16;
  _cf.data[4] = (*iThresh & 0x0000FF00) >> 8;
  _cf.data[5] = *iThresh  & 0x000000FF;
  _idSyncSend++;
  return _can->send(&_cf);
}

bool AddonShieldCAN::setPWMFrequency(unsigned char channel, unsigned short frequency)
{
  _cf.can_dlc = 4;
  _cf.data[0] = CMD_SET_FREQUENCY;
  _cf.data[1] = channel;
  _cf.data[2] = (frequency >> 8) & 0xFF;
  _cf.data[3] = frequency & 0xFF;
  _idSyncSend++;
  return _can->send(&_cf);
}

bool AddonShieldCAN::setPulseWidth(unsigned char channel, unsigned char pwm)
{
  _cf.can_dlc = 3;
  _cf.data[0] = CMD_SET_PULSEWIDTH;
  _cf.data[1] = channel;
  _cf.data[2] = pwm & 0xFF;
  _idSyncSend++;
  return _can->send(&_cf);
}

bool AddonShieldCAN::enable12V()
{
  _cf.can_dlc = 1;
  _cf.data[0] = CMD_CH3_SET_12V;
  _idSyncSend++;
  return _can->send(&_cf);
}

bool AddonShieldCAN::disable12V()
{
  _cf.can_dlc = 1;
  _cf.data[0] = CMD_CH3_SET_5V;
  _idSyncSend++;
  return _can->send(&_cf);
}

bool AddonShieldCAN::enable19V()
{
  _cf.can_dlc = 1;
  _cf.data[0] = CMD_CH4_SET_19V;
  _idSyncSend++;
  return _can->send(&_cf);
}

bool AddonShieldCAN::disable19V()
{
  _cf.can_dlc = 1;
  _cf.data[0] = CMD_CH4_SET_5V;
  _idSyncSend++;
  return _can->send(&_cf);
}
void AddonShieldCAN::notify(struct can_frame* frame)
{
  if(frame->can_dlc==5)
  {
    if(frame->data[0] == RESPONSE_VOLTAGE)
    {
      unsigned int voltage = (frame->data[1] | (frame->data[2] << 8) | (frame->data[3] << 16) | (frame->data[4] << 24));
      _voltage = ((float)voltage) / 100.f;
    }
    _idSyncReceive = _idSyncSend;
  }
}

bool AddonShieldCAN::waitForSync(unsigned int timeoutInMillis)
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
