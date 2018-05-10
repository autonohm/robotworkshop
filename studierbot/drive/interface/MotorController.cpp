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
  _params = params;

  cout << "Maximum RPM: " << _params.rpmMax << endl;
  cout << _params.kp << " " << _params.ki << " " << _params.kd << endl;
}

MotorController::~MotorController()
{

}

float MotorController::getRPMMax() const
{
  return _params.rpmMax;
}

float MotorController::getGearRatio() const
{
  return _params.gearRatio;
}
