#include "MotorControllerSerial.h"
#include <iostream>
#include <cmath>

MotorControllerSerial::MotorControllerSerial(MotorParams &params) : MotorController(params)
{

  _baud = B115200;
  _com  = new SerialPort(params.comPort.c_str(), _baud);

  bool  retval = false;
  float responseF;

  while(!retval)
  {
    _bufCmd[0] = 0x16;
    _bufCmd[13] = 'F';
    convertTo12ByteArray(_params.gearRatio, &_bufCmd[1]);
    int sent = _com->send(_bufCmd, 14);
    retval = _com->receive(_bufResponse, 13);
    convertFromByteArray(_bufResponse, responseF);
    retval = (_params.gearRatio==responseF);
  }
  std::cout << "Gear ratio: " << _params.gearRatio << std::endl;

  retval = false;

  while(!retval)
  {
    _bufCmd[0] = 0x17;
    _bufCmd[13] = 'F';
    convertTo12ByteArray(_params.encoderRatio, &_bufCmd[1]);
    int sent = _com->send(_bufCmd, 14);
    retval = _com->receive(_bufResponse, 13);
    convertFromByteArray(_bufResponse, responseF);
    retval = (_params.encoderRatio==responseF);
  }

  float checkF;
  int checkI;
  retval &= sendToMotorshieldF(0x02, _params.kp, &checkF, true);
  retval &= sendToMotorshieldF(0x03, _params.ki, &checkF, true);
  retval &= sendToMotorshieldF(0x04, _params.kd, &checkF, true);
  retval &= sendToMotorshieldI(0x15, _params.antiWindup, &checkI, true);

  if(!retval)
    std::cout << "WARNING: PID parameters could not be set. Motorcontroller cannot be enabled." << std::endl;

  _init = retval;

  stop();
}

MotorControllerSerial::~MotorControllerSerial()
{
  stop();
}

MotorParams MotorControllerSerial::getStandardParameters()
{
  MotorParams p;
  p.port         = std::string("/dev/frdm_dc_shield");
  return p;
}

bool MotorControllerSerial::enable()
{
  bool retval = false;
  if(_init)
  {
    int   responseI;
    // Enable Motorcontroller
    while(!retval)
    {
      sendToMotorshieldI(0x18, 1, &responseI, true);
      retval = (responseI==1);
    }
  }
  return retval;
}

bool setPWM(std::vector<int> pwm, std::vector<float> &rpm)
{
  return false;
}

bool MotorControllerSerial::setRPM(std::vector<float> rpmIn, std::vector<float> &rpmOut)
{
  bool retval = false;
  int len = rpmIn.size() > 6 ? 6 : rpmIn.size();
  float r[6];
  r[0] = 0.f;
  r[1] = 0.f;
  r[2] = 0.f;
  r[3] = 0.f;
  r[4] = 0.f;
  r[5] = 0.f;
  for(int i=0; i<len; i++)
    r[i] = rpmIn[i];

  float rpmLargest = std::abs(r[0]);
  for(int i=1; i<6; i++)
  {
    if(std::abs(r[i])> rpmLargest)
      rpmLargest = std::abs(r[i]);
  }
  float factor = rpmLargest / _params.rpmMax;

  if(factor>1.0)
  {
    for(int i=0; i<6; i++)
    {
      r[i] = r[i] /= factor;
    }
  }
  short wset[6];
  short wResponse[6];

  wset[0] = r[0] * VALUESCALE;
  wset[1] = r[1] * VALUESCALE;
  wset[2] = r[2] * VALUESCALE;
  wset[3] = r[3] * VALUESCALE;
  wset[4] = r[4] * VALUESCALE;
  wset[5] = r[5] * VALUESCALE;

  bool retval = sendToMotorshieldS(0x01, wset, &wResponse, true);

  if(retval)
  {
    _rpm[0] = wResponse[0];
    _rpm[1] = wResponse[1];
    _rpm[2] = wResponse[2];
    _rpm[3] = wResponse[3];
    _rpm[4] = wResponse[4];
    _rpm[5] = wResponse[5];

    rpmOut.clear();
    for(int i=0; i<len; i++)
      rpmOut.push_back(((float)_rpm[i]) / (float)VALUESCALE);

    retval = true;
  }
  else
    std::cout << "failed to receive" << std::endl;

  return retval;
}

void MotorControllerSerial::stop()
{
  short wset[6] = {0, 0, 0, 0, 0, 0};
  short wResponse[6];

  bool retval = sendToMotorshieldS(0x01, wset, &wResponse, true);
}

template<typename T>
bool MotorControllerSerial::sendToMotorshield(char cmd, T param, T* response, bool echo)
{
  _bufCmd[0] = cmd;
  convertTo12ByteArray(param, &_bufCmd[1]);
  int sent = _com->send(_bufCmd, 14);
  bool retval = _com->receive(_bufResponse, 13);

  T check;
  convertFromByteArray(_bufResponse, check);

  if(echo)
  {
    std::cout << "Sent " << param << ", echo: " << check << std::endl;
  }

  return (param==check);
}

bool MotorControllerSerial::sendToMotorshieldF(char cmd, float param, float* response, bool echo)
{
  _bufCmd[13] = 'F';
  return sendToMotorshield<float>(cmd, param, response, echo);
}

bool MotorControllerSerial::sendToMotorshieldI(char cmd, int param, int* response, bool echo)
{
  _bufCmd[13] = 'I';
  return sendToMotorshield<int>(cmd, param, response, echo);
}

bool MotorControllerSerial::sendToMotorshieldS(char cmd, short param[6], short (*response)[6], bool echo)
{
  _bufCmd[13] = 'S';
  return sendToMotorshield<short[6]>(cmd, param, response, false);
}
