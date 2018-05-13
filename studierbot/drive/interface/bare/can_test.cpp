/**
 * @author Stefan May
 * @date 08.05.2018
 * @brief Test program for CAN interface
 */
#include "MotorControllerCAN.h"
#include <unistd.h>
#include <iostream>
#include <cmath>

using namespace std;

void setPWM(MotorControllerCAN* mc, int val)
{
  int pwm[2];
  float rpm[2];
  pwm[0] = val;
  pwm[1] = val;
  if(mc->setPWM(pwm))
  {
    if(mc->waitForSync())
    {
      mc->getRPM(rpm);
      std::cout << "RPM1: " << rpm[0] << " " << ", RPM2: " << rpm[1] << std::endl;
    }
    else
    {
      std::cout << "Error synchronizing with device" << std::endl;
    }
  }
  else
  {
    std::cout << "Failed to set PWM value" << std::endl;
    usleep(1000);
  }
}


int main(int argc, char* argv[])
{
  SocketCAN can(std::string("slcan0"));
  MotorControllerCAN mc(&can, 0);
  can.startListener();

  if(!mc.enable())
  {
    std::cout << "Enabling motor controller failed" << std::endl;
    return -1;
  }
  float gearRatio[2]          = {1024.f, 0.f};
  float encoderTicksPerRev[2] = {14.f, 0.f};
  if(!mc.setGearRatio(gearRatio))
  {
    std::cout << "Setting gear ratio failed" << std::endl;
    return -1;
  }
  if(!mc.setEncoderTicksPerRev(encoderTicksPerRev))
  {
    std::cout << "Setting encoder parameters failed" << std::endl;
    return -1;
  }

  for(int i=0; i<500; i++)
  {
    float phase = ((float)i) * (2.f*M_PI) * 0.002;
    int val = (int)(sin(phase) * 100.f);
    setPWM(&mc, val);
  }
}
