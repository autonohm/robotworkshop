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
  _rpmMax = RPMMAX;
  _cmdMax = CMDMAX;
  _baud = B115200;
  _comPort = "/dev/ttyACM0";

  _maxCmd = CMDMAX;
  _minCmd = -CMDMAX;

  _kp = 3.1f;
  _ki = 0.0f;
  _kd = 0.0f;

  _com = new SerialPort(_comPort.c_str(), _baud);

  init();
  stop();
}

/**
 * Send float commands to motor shield
 * @param cmd command byte
 * @param param float parameter
 * @param echo verbosity of function, true provides command line output
 */
template<typename T>
  bool Motorcontroller::sendToMotorshield(char cmd, T param, bool echo)
  {
    _bufCmd[0] = cmd;
    convertTo12ByteArray(param, &_bufCmd[1]);
    int sent = _com->send(_bufCmd, 14);
    bool retval = _com->receive(_bufIn, 13);

    if(echo)
    {
      T check;
      convertFromByteArray(_bufIn, check);
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
    convertFromByteArray(_bufIn, check);
    //cout << "Sent " << param[0] << " / " << param[1] << ", echo: " << check[0] << " / " << check[1] << endl;
  }
  return retval;
}

void Motorcontroller::init()
{

  // parasitic time constant of closed-loop controller implemented in motor shield
  float tPar = 0.01;
  if(EULER)
  {
    float aTf[4];
    float bTf[4];

    pidToTransferFunction(_kp, _ki, _kd, tPar, bTf, aTf, true);

    float A[9];
    float b[3];
    float c[3];
    float d;

    transferFunctionToStateControl(bTf, aTf, second, A, b, c, d, true);

    for(int i = 0; i < 9; i++)
      sendToMotorshieldF(0x05 + i, A[i], true);

    for(int i = 0; i < 3; i++)
      sendToMotorshieldF(0x0E + i, b[i], true);

    for(int i = 0; i < 3; i++)
      sendToMotorshieldF(0x11 + i, c[i], true);

    sendToMotorshieldF(0x14, d, true);
  }
  else
  {
    sendToMotorshieldF(0x02, _kp, true);
    sendToMotorshieldF(0x03, _ki, true);
    sendToMotorshieldF(0x04, _kd, true);
    sendToMotorshieldI(0x15, ANTIWINDUP, true);
  }
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
//begin
  short wset[2];

  wset[0] = rpmLeft * VALUESCALE;
  wset[1] = -rpmRight * VALUESCALE;

  bool retval = sendToMotorshieldS(0x01, wset, true);

  if(retval)
  {
    short rpm1 = ((_bufIn[0] << 8) & 0xFF00) | (_bufIn[1] & 0x00FF);
    short rpm2 = ((_bufIn[3] << 8) & 0xFF00) | (_bufIn[2] & 0x00FF);

    cout << "rpm1: " << rpm1 / VALUESCALE << " rpm2: " << rpm2 / VALUESCALE << endl;
  }
  else
    cout << "failed to receive" << endl;

  //end

//  _encoder.setForwardLeft(rpmLeft>=0.0);
//  _encoder.setForwardRight(rpmRight>=0.0);
}

double Motorcontroller::getRPMLeft(double* dt)
{
  return 0;
  /*
   double ticks;
   _encoder.getTicksLeft(&ticks, dt);
   return ticks / (_encoder.getTicksPerTurn() * *dt) * 60.0;
   */
}

double Motorcontroller::getRPMRight(double* dt)
{
  return 0;
  /*
   double ticks;
   _encoder.getTicksRight(&ticks, dt);
   return ticks / (_encoder.getTicksPerTurn() * *dt) * 60.0;
   */
}

void Motorcontroller::stop()
{
  short wset[6] = {0, 0, 0, 0, 0, 0};

  bool retval = sendToMotorshieldS(0x01, wset, true);
}
