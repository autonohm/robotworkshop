#include "Motorcontroller.h"
#include <unistd.h>

#include "params.h"

#include <termios.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

#include <iostream>
using namespace std;

Motorcontroller::Motorcontroller()
{
	_rpmMax      = RPMMAX;
	_cmdMax      = CMDMAX;

	_maxCmd = CMDMAX;
	_minCmd = -CMDMAX;

  struct termios options;
  _tty_fd=open("/dev/ttyAMA0", O_RDWR | O_NONBLOCK | O_NOCTTY | O_NDELAY);

  usleep(1000);

  tcgetattr(_tty_fd, &options);
  cfsetospeed(&options,B19200);            // baudrate
  cfsetispeed(&options,B19200);            // baudrate

  stop();
}

Motorcontroller::~Motorcontroller()
{
  stop();
}

int Motorcontroller::getRPMMax()
{
  return (int)_rpmMax;
}

void Motorcontroller::setRPM(double rpmLeft, double rpmRight)
{
  if(abs(rpmRight) > _rpmMax || abs(rpmLeft) > _rpmMax)
  {
     double rpmLargest = abs(rpmRight);
     if(rpmLargest<abs(rpmLeft)) rpmLargest = abs(rpmLeft);
     double factor = rpmLargest / _rpmMax;
     rpmLeft /= factor;
     rpmRight /= factor;
  } 
  
  // situations below should not happen
  // ToDo: check if it is obsolete
  if(rpmRight >  _rpmMax) rpmRight =  _rpmMax;
  if(rpmRight < -_rpmMax) rpmRight = -_rpmMax;
  if(rpmLeft >  _rpmMax) rpmLeft =  _rpmMax;
  if(rpmLeft < -_rpmMax) rpmLeft = -_rpmMax;

  int right = _cmdMax * rpmRight/_rpmMax;
  int left  = _cmdMax * rpmLeft/_rpmMax;
 
  cout << "RPM(target): " << rpmRight << " Cmd: " << right << endl;

  unsigned char c[2];
  if(left>0)
  {
    c[0] = 0xC2;
    c[1] = left;
  }
  else
  {
    c[0] = 0xC1;
    c[1] = -left;
  }
  write(_tty_fd,c,2);
  usleep(100);

  if(right>0)
  {
    c[0] = 0xCA;
    c[1] = right;
  }
  else
{
    c[0] = 0xCD;
    c[1] = -right;
  }
  write(_tty_fd,c,2);
  usleep(1000);

  _encoder.setForwardLeft(rpmLeft>=0.0);
  _encoder.setForwardRight(rpmRight>=0.0);
}

double Motorcontroller::getRPMLeft(double* dt)
{
  double ticks;
  _encoder.getTicksLeft(&ticks, dt);
  return ticks / (_encoder.getTicksPerTurn() * *dt) * 60.0;
}

double Motorcontroller::getRPMRight(double* dt)
{
  double ticks;
  _encoder.getTicksRight(&ticks, dt);
  return ticks / (_encoder.getTicksPerTurn() * *dt) * 60.0;
}

void Motorcontroller::stop()
{
  unsigned char c[2];
  c[0] = 0xC2;
  c[1] = 0x00;
  write(_tty_fd,c,2);
  usleep(1000);
  c[0] = 0xCA;
  c[1] = 0x00;
  write(_tty_fd,c,2);
  usleep(1000);
}
