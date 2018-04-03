#include "MotorController.h"

#include <unistd.h>

#include <termios.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <cmath>

#include <iostream>

using namespace std;

MotorController::MotorController(MotorParams &params)
{
  _rpmMax       = params.rpmMax;
  _gearRatio    = params.gearRatio;
  _encoderRatio = params.encoderRatio;
  _kp           = params.kp;
  _ki           = params.ki;
  _kd           = params.kd;
  _antiWindup   = params.antiWindup;

  cout << "Maximum RPM: " << _rpmMax << endl;
  cout << _kp << " " << _ki << " " << _kd << endl;
}

MotorController::~MotorController()
{

}

float MotorController::getRPMMax() const
{
  return _rpmMax;
}

float MotorController::getGearRatio() const
{
  return _gearRatio;
}
