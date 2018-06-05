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
  pwm[0] = val;
  pwm[1] = val;
  if(!mc->setPWM(pwm))
  {
    std::cout << "Failed to set PWM value for channel" << mc->getChannel() << std::endl;
    usleep(1000);
  }
}

void waitForSync(MotorControllerCAN* mc)
{
  float rpm[2];
  if(mc->waitForSync())
  {
    mc->getRPM(rpm);
    std::cout << "Channel" << mc->getChannel() << " RPM1: " << rpm[0] << " " << ", RPM2: " << rpm[1] << std::endl;
  }
  else
  {
    std::cout << "Error synchronizing with device" << mc->getChannel() << std::endl;
  }
}


int main(int argc, char* argv[])
{
  SocketCAN can(std::string("slcan0"));
  MotorControllerCAN mc0(&can, 0);
  MotorControllerCAN mc1(&can, 1);
  MotorControllerCAN mc2(&can, 2);
  MotorControllerCAN mc3(&can, 3);
  std::vector<MotorControllerCAN*> mc;
  mc.push_back(&mc0);
  mc.push_back(&mc1);
  mc.push_back(&mc2);
  mc.push_back(&mc3);

  float gearRatio[2]          = {131.f, 131.f};
  float encoderTicksPerRev[2] = {64.f, 64.f};

  can.startListener();

  int dev = 0;
  for(dev=0; dev<4; dev++)
  {
    if(!mc[dev]->enable())
    {
      std::cout << "Enabling motor controller failed of device " << dev << std::endl;
      return -1;
    }
    if(!mc[dev]->setGearRatio(gearRatio))
    {
      std::cout << "Setting gear ratio failed of device " << dev << std::endl;
      return -1;
    }
    if(!mc[dev]->setEncoderTicksPerRev(encoderTicksPerRev))
    {
      std::cout << "Setting encoder parameters failed of device " << dev << std::endl;
      return -1;
    }
    usleep(25000);
  }

  for(int i=0; i<500; i++)
  {
    float phase = ((float)i) * (2.f*M_PI) * 0.004;
    int val = (int)(sin(phase) * 100.f);
    for(dev=0; dev<4; dev++)
      setPWM(mc[dev], val);
    for(dev=0; dev<4; dev++)
      waitForSync(mc[dev]);
  }
}
