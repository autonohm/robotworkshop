#include "MotorControllerCAN.h"
#include <iostream>
#include <cmath>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

MotorControllerCAN::MotorControllerCAN(MotorParams &params) : MotorController(params)
{
  init();
  stop();
}

MotorControllerCAN::~MotorControllerCAN()
{
  stop();
}

void MotorControllerCAN::init()
{
  openPort("slcan0");

  /*bool  retval = false;
  float responseF;

  while(!retval)
  {
    _bufCmd[0]  = 0x16;
    _bufCmd[13] = 'F';
    convertTo12ByteArray(_gearRatio, &_bufCmd[1]);
    int sent = _com->send(_bufCmd, 14);
    //retval = _com->receive(_bufResponse, 13);
    //convertFromByteArray(_bufResponse, responseF);
    retval = (_gearRatio==responseF);
  }
  std::cout << "Gear ratio: " << _gearRatio << std::endl;

  retval = false;

  while(!retval)
  {
    _bufCmd[0]  = 0x17;
    _bufCmd[13] = 'F';
    convertTo12ByteArray(_encoderRatio, &_bufCmd[1]);
    int sent    = _com->send(_bufCmd, 14);
    //retval      = _com->receive(_bufResponse, 13);
    //convertFromByteArray(_bufResponse, responseF);
    retval      = (_encoderRatio==responseF);
  }*/

  //sendToMotorshieldF(0x02, _kp, true);
  //sendToMotorshieldF(0x03, _ki, true);
  //sendToMotorshieldF(0x04, _kd, true);
  //sendToMotorshieldI(0x15, _antiWindup, true);
}

void MotorControllerCAN::setRPM(float rpm[6])
{
  float rpmLargest = std::abs(rpm[0]);
  for(int i=1; i<6; i++)
  {
    if(std::abs(rpm[i])> rpmLargest)
      rpmLargest = std::abs(rpm[i]);
  }
  float factor = rpmLargest / _rpmMax;

  if(factor>1.0)
  {
    for(int i=0; i<6; i++)
    {
      rpm[i] = rpm[i] /= factor;
    }
  }
  short wset[6];

  wset[0] = rpm[0] * VALUESCALE;
  wset[1] = rpm[1] * VALUESCALE;
  wset[2] = rpm[2] * VALUESCALE;
  wset[3] = rpm[3] * VALUESCALE;
  wset[4] = rpm[4] * VALUESCALE;
  wset[5] = rpm[5] * VALUESCALE;

  bool retval = false;

  if(retval)
  {

  }
  else
    std::cout << "failed to receive" << std::endl;
}

float MotorControllerCAN::getRPM(unsigned int idx)
{
  return 0.f;
}

void MotorControllerCAN::stop()
{
  short wset[6] = {0, 0, 0, 0, 0, 0};
}

int MotorControllerCAN::openPort(const char *port)
{
  struct ifreq ifr;
  struct sockaddr_can addr;

  _soc = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if(_soc < 0)
  {
    return -1;
  }

  addr.can_family = AF_CAN;
  strcpy(ifr.ifr_name, port);

  if (ioctl(_soc, SIOCGIFINDEX, &ifr) < 0)
  {
    return -1;
  }

  addr.can_ifindex = ifr.ifr_ifindex;

  fcntl(_soc, F_SETFL, O_NONBLOCK);

  if (bind(_soc, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {

    return -1;
  }

  return 0;
}

int MotorControllerCAN::sendPort(struct can_frame *frame)
{
  int retval;
  retval = write(_soc, frame, sizeof(struct can_frame));
  if (retval != sizeof(struct can_frame))
  {
    return -1;
  }
  else
  {
    return 0;
  }
}

void MotorControllerCAN::readPort()
{
  struct can_frame frame_rd;
  int recvbytes = 0;

  int read_can_port = 1;
  while(read_can_port)
  {
    struct timeval timeout = {1, 0};
    fd_set readSet;
    FD_ZERO(&readSet);
    FD_SET(_soc, &readSet);

    if (select((_soc + 1), &readSet, NULL, NULL, &timeout) >= 0)
    {
      if (!read_can_port)
      {
        break;
      }
      if (FD_ISSET(_soc, &readSet))
      {
        recvbytes = read(_soc, &frame_rd, sizeof(struct can_frame));
        if(recvbytes)
        {
          std::cout << "dlc = " << frame_rd.can_dlc << ", data = " << frame_rd.data << std::endl;
        }
      }
    }

  }
}

int MotorControllerCAN::closePort()
{
  close(_soc);
  return 0;
}
