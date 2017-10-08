#include "Motorcontroller.h"

#include <unistd.h>

#include <termios.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <cmath>

#include <iostream>

#include "params.h"
using namespace std;

Motorcontroller::Motorcontroller()
{
  _rpmMax    = RPMMAX;
  _gearRatio = GEARRATIO;
  _baud      = B115200;
  _comPort   = "/dev/ttyACM0";

  _kp = PID_KP;
  _ki = PID_KI;
  _kd = PID_KD;

  _com = new SerialPort(_comPort.c_str(), _baud);

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
  _gearRatio   = GEARRATIO;
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
  _encoderRatio = ENCODERRATIO;
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
  sendToMotorshieldI(0x15, ANTIWINDUP, true);
}

Motorcontroller::~Motorcontroller()
{
  stop();
}

int Motorcontroller::getRPMMax()
{
  return (int)_rpmMax;
}

double Motorcontroller::getGearRatio() const
{
  return _gearRatio;
}

void Motorcontroller::setRPM(double rpmLeft, double rpmRight)
{
  if(std::abs(rpmRight) > _rpmMax || std::abs(rpmLeft) > _rpmMax)
  {
    double rpmLargest = std::abs(rpmRight);
    if(rpmLargest < std::abs(rpmLeft))
    {
      rpmLargest = std::abs(rpmLeft);
    }

    double factor = rpmLargest / _rpmMax;
    rpmLeft /= factor;
    rpmRight /= factor;
  }

  short wset[6];

  wset[0] = rpmLeft * VALUESCALE;
  wset[1] = rpmRight * VALUESCALE;
  wset[2] = wset[0];
  wset[3] = wset[1];
  wset[4] = wset[0];
  wset[5] = wset[1];

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

double Motorcontroller::getRPM(unsigned int idx)
{
  return ((double)_rpm[idx]) / (double)VALUESCALE;
}

void Motorcontroller::stop()
{
  short wset[6] = {0, 0, 0, 0, 0, 0};

  bool retval = sendToMotorshieldS(0x01, wset, true);
}
