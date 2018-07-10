#include "SocketCAN.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <iostream>
#include <unistd.h>
#include <string.h>


SocketCAN::SocketCAN(std::string devFile)
{
  _soc = 0;
  _listenerIsRunning = false;
  _shutDownListener  = false;

  if(!openPort(devFile.c_str()))
    std::cout << "WARNING: Cannot open CAN device interface: " << devFile << std::endl;
}

SocketCAN::~SocketCAN()
{
  closePort();
}

bool SocketCAN::registerObserver(SocketCANObserver* observer)
{
  _mutex.lock();
  _observers.push_back(observer);
  _mutex.unlock();
  return true;
}

bool SocketCAN::openPort(const char *port)
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

bool SocketCAN::send(struct can_frame* frame)
{
  int retval;
  _mutex.lock();
  retval = write(_soc, frame, sizeof(struct can_frame));
  _mutex.unlock();
  if (retval != sizeof(struct can_frame))
  {
    std::cout << "Can transmission error for command " << (int)(frame->data[0]) << ", returned " << retval << " submitted bytes instead of " << sizeof(struct can_frame) << std::endl;
    return false;
  }
  else
  {
    return true;
  }
}

bool SocketCAN::startListener()
{
  if(_listenerIsRunning) return false;

  _thread = new std::thread(&SocketCAN::listener, this);

  while(!_listenerIsRunning)
    usleep(100);

  return true;
}

bool SocketCAN::listener()
{
  _shutDownListener  = false;

  struct can_frame frame_rd;
  int recvbytes = 0;

  struct timeval timeout = {0, 100};
  fd_set readSet;

  std::cout << "# Listener start" << std::endl;

  _listenerIsRunning = true;
  while(!_shutDownListener)
  {
    FD_ZERO(&readSet);

    _mutex.lock();
    FD_SET(_soc, &readSet);
    if (select((_soc + 1), &readSet, NULL, NULL, &timeout) >= 0)
    {
      if (FD_ISSET(_soc, &readSet))
      {
        recvbytes = read(_soc, &frame_rd, sizeof(struct can_frame));
        if(recvbytes)
        {
          for(std::vector<SocketCANObserver*>::iterator it=_observers.begin(); it!=_observers.end(); ++it)
          {
            if((*it)->getCANId()==frame_rd.can_id)
              (*it)->notify(&frame_rd);
          }
        }
      }
    }
    _mutex.unlock();

    usleep(100);
  }
  std::cout << "# Listener stop" << std::endl;

  _listenerIsRunning = false;
  return true;
}

void SocketCAN::stopListener()
{
  _shutDownListener = true;

  _thread->join();

  delete _thread;
}

bool SocketCAN::closePort()
{
  bool retval = false;
  if(_soc)
  {
    retval = (close(_soc)==0);
  }
  return retval;
}

