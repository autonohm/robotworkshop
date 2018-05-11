#include "MotorControllerCAN.h"
#include <iostream>
#include <cmath>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <iomanip>

#define GROUPID   (0x01 << 9)
#define SYSTEMID  (0x4  << 6)
#define COMPINPUT (0x0  << 5)

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
  _cf.can_id  = 0x31F;
  _cf.can_dlc = 1;
  _cf.data[0] = 0x01;
  sendPort(&_cf);
}

/**
 * Set pulse width modulated signal
 * @param[in] rpm pulse width in range [-100;100]
 * @param[out] revolutions per minute (RPM)
 * @return success
 */
bool MotorControllerCAN::setPWM(std::vector<int> pwm, std::vector<float> &rpm)
{
  bool retval = false;

  if(pwm.size()%2==0)
  {
    for(int i=0; i<pwm.size(); i+=2)
    {
      _cf.can_id  = GROUPID | SYSTEMID | COMPINPUT | 0x0;
      _cf.can_dlc = 3;

      int vel1  = (((int)pwm[2*i]) * 127) / 100;
      int vel2 = (((int)pwm[2*i+1]) * 127) / 100;
      //std::cout << vel1 << " " << vel2 << std::endl;
      _cf.data[0] = 0x10;
      _cf.data[1] = (char)(vel1 + 0x7F);
      _cf.data[2] = (char)(vel2 + 0x7F);

      sendPort(&_cf);
      rpm.clear();
      float rpm1;
      float rpm2;
      retval = readPort(&rpm1, &rpm2);
      if(retval)
      {

        std::cout << std::setw(6) << vel1 << " " << std::setw(6) << vel2 << " " << std::setw(6) << rpm1 << " " << std::setw(6) << rpm2 << std::endl;
        rpm.push_back(rpm1);
        rpm.push_back(rpm2);
      }
    }
  }

  return retval;
}

bool MotorControllerCAN::setRPM(std::vector<float> rpmIn, std::vector<float> &rpmOut)
{
  return false;
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

bool MotorControllerCAN::readPort(float* rpm1, float* rpm2)
{
  bool retval = false;

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
          *rpm1 = ((float)((short)(frame_rd.data[1] | (frame_rd.data[2] << 8))))/100.f;
          *rpm2 = ((float)((short)(frame_rd.data[3] | (frame_rd.data[4] << 8))))/100.f;
          retval = true;
        }
      }
    }
  }
  return retval;
}

int MotorControllerCAN::closePort()
{
  if(_soc)
    close(_soc);
  return 0;
}
