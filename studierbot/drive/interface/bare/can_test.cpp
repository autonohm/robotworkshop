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
    std::cout << "# Failed to set PWM value for channel" << mc->getChannel() << std::endl;
    usleep(1000);
  }
}

void setRPM(MotorControllerCAN* mc, float val)
{
  float rpm[2];
  rpm[0] = val;
  rpm[1] = val;
  if(!mc->setRPM(rpm))
  {
    std::cout << "# Failed to set RPM value for channel" << mc->getChannel() << std::endl;
    usleep(1000);
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

  float gearRatio[2]            = {131.f, 3*24.f};
  float encoderTicksPerRev[2]   = {64.f,  20.f};
  unsigned short frequencyScale = 32;   // PWM frequency: 1/frequencyScale x 500kHz
  unsigned char maxPulse        = 64;   // Set maxPulse to 127 to apply full power
  float inputWeight             = 0.8f; // Smoothing parameter for input values: smoothVal = inputWeight x prevVal + (1.f-inputWeight) x newVal
  float kp                      = 2.f;
  float ki                      = 200.f;
  float kd                      = 0.f;

  can.startListener();

  unsigned int dev = 0;
  for(dev=0; dev<mc.size(); dev++)
  {
    if(!mc[dev]->setFrequencyScale(frequencyScale))
    {
      std::cout << "# Setting frequency scaling parameter failed for device " << dev << std::endl;
      return -1;
    }
    if(!mc[dev]->enable())
    {
      std::cout << "# Enabling motor controller failed for device " << dev << std::endl;
      return -1;
    }
    if(!mc[dev]->setMaxPulseWidth(maxPulse))
    {
      std::cout << "# Setting maximum pulse width failed for device " << dev << std::endl;
      return -1;
    }
    if(!mc[dev]->setGearRatio(gearRatio))
    {
      std::cout << "# Setting gear ratio failed for device " << dev << std::endl;
      return -1;
    }
    if(!mc[dev]->setEncoderTicksPerRev(encoderTicksPerRev))
    {
      std::cout << "# Setting encoder parameters failed for device " << dev << std::endl;
      return -1;
    }
    if(!mc[dev]->setKp(kp))
    {
      std::cout << "# Setting proportional factor of PID controller failed for device " << dev << std::endl;
      return -1;
    }
    if(!mc[dev]->setKi(ki))
    {
      std::cout << "# Setting integration factor of PID controller failed for device " << dev << std::endl;
      return -1;
    }
    if(!mc[dev]->setKd(kd))
    {
      std::cout << "# Setting differential factor of PID controller failed for device " << dev << std::endl;
      return -1;
    }
    if(!mc[dev]->setInputWeight(inputWeight))
    {
      std::cout << "# Setting differential factor of PID controller failed for device " << dev << std::endl;
      return -1;
    }
    usleep(25000);
  }

  for(int i=0; i<500; i++)
  {
    float phase = ((float)i) * (2.f*M_PI) * 0.002;
    float amplitude = 40.f;
    float val = (sin(phase) * amplitude);
    //float val = amplitude;
    for(dev=0; dev<mc.size(); dev++)
      setPWM(mc[dev], val);
      //setRPM(mc[dev], val);

    std::cout << val;
    for(dev=0; dev<mc.size(); dev++)
    {
      if(mc[dev]->waitForSync())
      {
        float rpm[2];
        mc[dev]->getRPM(rpm);
        std::cout << " " << rpm[0] << " " << rpm[1];
      }
      else
      {
        std::cout << "# Error synchronizing with device" << mc[dev]->getChannel() << std::endl;
      };
    }
    std::cout << std::endl;
  }
}
