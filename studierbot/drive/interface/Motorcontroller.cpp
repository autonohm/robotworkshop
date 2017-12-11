#include "Motorcontroller.h"

#include <unistd.h>

#include <termios.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <cmath>

#include <iostream>

using namespace std;

Motorcontroller::Motorcontroller(MotorParams params)
{
  _rpmMax       = params.rpmMax;
  _gearRatio    = params.gearRatio;
  _encoderRatio = params.encoderRatio;
  _antiWindup   = params.antiWindup;
  _kp           = params.kp;
  _ki           = params.ki;
  _kd           = params.kd;

  cout << "Maximum RPM: " << _rpmMax << endl;
  cout << _kp << " " << _ki << " " << _kd << endl;

  _baud         = B115200;
  _com = new SerialPort(params.comPort.c_str(), _baud);

  init();
  stop();
}

template<typename T>
bool Motorcontroller::sendToMotorshield(char cmd, T param, bool echo)
{
  _bufCmd[0] = cmd;
  convertTo12ByteArray(param, &_bufCmd[1]);
  int sent = _com->send(_bufCmd, 14);
  bool retval = _com->receive(_bufResponse, 13);

  if(echo)
  {
    T check;
    convertFromByteArray(_bufResponse, check);
    cout << "Sent " << param << ", echo: " << check << endl;
  }

  return retval;
}

bool Motorcontroller::sendToMotorshieldF(char cmd, float param, bool echo)
{
  _bufCmd[13] = 'F';
  return sendToMotorshield<float>(cmd, param, echo);
}

bool Motorcontroller::sendToMotorshieldI(char cmd, int param, bool echo)
{
  _bufCmd[13] = 'I';
  return sendToMotorshield<int>(cmd, param, echo);
}

bool Motorcontroller::sendToMotorshieldS(char cmd, short param[6], bool echo)
{
  _bufCmd[13] = 'S';
  bool retval = sendToMotorshield<short[6]>(cmd, param, false);
  if(echo)
  {
    short check[6];
    convertFromByteArray(_bufResponse, check);
  }
  return retval;
}

void Motorcontroller::init()
{
  bool  retval = false;
  float responseF;

  while(!retval)
  {
    _bufCmd[0] = 0x16;
    _bufCmd[13] = 'F';
    convertTo12ByteArray(_gearRatio, &_bufCmd[1]);
    int sent = _com->send(_bufCmd, 14);
    retval = _com->receive(_bufResponse, 13);
    convertFromByteArray(_bufResponse, responseF);
    retval = (_gearRatio==responseF);
  }
  cout << "Gear ratio: " << _gearRatio << endl;

  retval = false;

  while(!retval)
  {
    _bufCmd[0] = 0x17;
    _bufCmd[13] = 'F';
    convertTo12ByteArray(_encoderRatio, &_bufCmd[1]);
    int sent = _com->send(_bufCmd, 14);
    retval = _com->receive(_bufResponse, 13);
    convertFromByteArray(_bufResponse, responseF);
    retval = (_encoderRatio==responseF);
  }

  sendToMotorshieldF(0x02, _kp, true);
  sendToMotorshieldF(0x03, _ki, true);
  sendToMotorshieldF(0x04, _kd, true);
  sendToMotorshieldI(0x15, _antiWindup, true);
}

Motorcontroller::~Motorcontroller()
{
  stop();
}

float Motorcontroller::getRPMMax()
{
  return _rpmMax;
}

float Motorcontroller::getGearRatio() const
{
  return _gearRatio;
}

void Motorcontroller::setRPM(float rpm[6])
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

  bool retval = sendToMotorshieldS(0x01, wset, true);

  if(retval)
  {
    _rpm[0] = ((_bufResponse[0] << 8) & 0xFF00) | (_bufResponse[1] & 0x00FF);
    _rpm[1] = ((_bufResponse[2] << 8) & 0xFF00) | (_bufResponse[3] & 0x00FF);
    _rpm[2] = ((_bufResponse[4] << 8) & 0xFF00) | (_bufResponse[5] & 0x00FF);
    _rpm[3] = ((_bufResponse[6] << 8) & 0xFF00) | (_bufResponse[7] & 0x00FF);
    _rpm[4] = ((_bufResponse[8] << 8) & 0xFF00) | (_bufResponse[9] & 0x00FF);
    _rpm[5] = ((_bufResponse[10] << 8) & 0xFF00) | (_bufResponse[11] & 0x00FF);
  }
  else
    cout << "failed to receive" << endl;
}

float Motorcontroller::getRPM(unsigned int idx)
{
  return ((float)_rpm[idx]) / (float)VALUESCALE;
}

void Motorcontroller::stop()
{
  short wset[6] = {0, 0, 0, 0, 0, 0};

  bool retval = sendToMotorshieldS(0x01, wset, true);
}
