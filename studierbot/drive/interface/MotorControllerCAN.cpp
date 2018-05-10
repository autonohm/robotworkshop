#include "MotorControllerCAN.h"
#include <iostream>
#include <cmath>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <net/if.h>

MotorControllerCAN::MotorControllerCAN(MotorParams &params) : MotorController(params)
{
  _soc = 0;

  if(!openPort(params.port.c_str()))
    std::cout << "WARNING: Cannot open CAN device interface" << std::endl;
}

MotorControllerCAN::~MotorControllerCAN()
{
  stop();
  closePort();
}

MotorParams MotorControllerCAN::getStandardParameters()
{
  MotorParams p;
  p.port = std::string("slcan0");
  return p;
}

bool MotorControllerCAN::enable()
{
  _cf.can_id  = 0x308;
  _cf.can_dlc = 1;
  _cf.data[0] = 0x01;
  sendPort(&_cf);
}

void MotorControllerCAN::setRPM(std::map<MotorControllerChannel, float> rpm)
{
  _cf.can_id  = 0x308;
  _cf.can_dlc = 3;

  //for(int i=0; i<rpm.size(); i++)
  {
    int vel = (int)rpm[CH0] + 0x7F;
    _cf.data[0] = 0x10;
    _cf.data[1] = (char)vel;
    _cf.data[2] = (char)vel;

    sendPort(&_cf);
    readPort();
  }
}

float MotorControllerCAN::getRPM(unsigned int idx)
{
  return 0.f;
}

void MotorControllerCAN::stop()
{
  _cf.can_id  = 0x308;
  _cf.can_dlc = 3;

  _cf.data[0] = 0x10;
  _cf.data[1] = 0x7F;
  _cf.data[2] = 0x7F;
  sendPort(&_cf);
}

bool MotorControllerCAN::openPort(const char *port)
{
  struct ifreq ifr;
  struct sockaddr_can addr;

  _soc = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if(_soc < 0)
  {
    return false;
  }

  addr.can_family = AF_CAN;
  strcpy(ifr.ifr_name, port);

  if (ioctl(_soc, SIOCGIFINDEX, &ifr) < 0)
  {
    return false;
  }

  addr.can_ifindex = ifr.ifr_ifindex;

  fcntl(_soc, F_SETFL, O_NONBLOCK);

  if (bind(_soc, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    return false;
  }

  return true;
}

bool MotorControllerCAN::sendPort(struct can_frame *frame)
{
  int retval;
  retval = write(_soc, frame, sizeof(struct can_frame));
  if (retval != sizeof(struct can_frame))
  {
    return false;
  }
  else
  {
    return true;
  }
}

void MotorControllerCAN::readPort()
{
  struct can_frame frame_rd;
  int recvbytes = 0;

  struct timeval timeout = {0, 100};
  fd_set readSet;
  FD_ZERO(&readSet);
  FD_SET(_soc, &readSet);

  if (select((_soc + 1), &readSet, NULL, NULL, &timeout) >= 0)
  {
    if (FD_ISSET(_soc, &readSet))
    {
      recvbytes = read(_soc, &frame_rd, sizeof(struct can_frame));
      if(recvbytes)
      {
        if(frame_rd.can_dlc==5)
        {
          int pos1 = frame_rd.data[1] | (frame_rd.data[2] << 8);
          int pos2 = frame_rd.data[3] | (frame_rd.data[4] << 8);
          std::cout << "Pos1: " << pos1 << " ,Pos2: " << pos2 << std::endl;
        }
      }
    }
  }
}

int MotorControllerCAN::closePort()
{
  if(_soc)
    close(_soc);
  return 0;
}
