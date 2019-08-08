/**
 * @author Stefan May
 * @date 01.12.2018
 * @brief Test program for CAN interface of add-on board
 */
#include "AddonShieldCAN.h"
#include <unistd.h>
#include <iostream>
#include <cmath>

using namespace std;

int main(int argc, char* argv[])
{
  SocketCAN can(std::string("slcan0"));
  can.startListener();

  AddonShieldCAN shield(&can);

  float servoFrequency = 330.f;
  float uLower = 500.f;
  float uUpper = 2500.f;
  shield.setPWMFrequency(2, (int)servoFrequency);
  shield.enable(2);
  shield.enable(3);
  shield.enable(4);

  // Voltage threshold (automatic switch-off)
  shield.setThreshold(2, 8.f);
  shield.setThreshold(3, 13.f);
  shield.setThreshold(4, 0.f);

  int lowerPoint = (int)(uLower / ((1.f / servoFrequency) * 1000000.f) * 100.f - 0.5f);
  int upperPoint = (int)(uUpper / ((1.f / servoFrequency) * 1000000.f) * 100.f + 0.5f);

  shield.setPulseWidth(2, lowerPoint);
  usleep(2000000);

  for(int i=lowerPoint; i<upperPoint;)
  {
    shield.setPulseWidth(2, i);

    if(shield.waitForSync(100))
    {
      i++;
      cout << "Pulse-width: " << i << ", Voltage: " << shield.getVoltage() << " V" << endl;
      usleep(100000);
    }
  }

  shield.disable(2);
  shield.disable(3);
  can.stopListener();

}
