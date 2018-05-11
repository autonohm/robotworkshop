/**
 * @author Stefan May
 * @date 08.05.2018
 * @brief Test program for CAN interface
 */
#include "MotorControllerCAN.h"
#include <unistd.h>

using namespace std;

int main(int argc, char* argv[])
{
  MotorParams params = MotorControllerCAN::getStandardParameters();
  MotorControllerCAN mc(params);
  mc.enable();

  std::vector<int> pwm;
  std::vector<float> rpmOut;

  pwm.resize(2);
  for(char i=0; i<100; i++)
  {
    pwm[0] = i;
    pwm[1] = i;
    mc.setPWM(pwm, rpmOut);
    usleep(40000);
  }
  for(char i=100; i>-100; i--)
  {
    pwm[0] = i;
    pwm[1] = i;
    mc.setPWM(pwm, rpmOut);
    usleep(40000);
  }
  for(char i=-100; i<=0; i++)
  {
    pwm[0] = i;
    pwm[1] = i;
    mc.setPWM(pwm, rpmOut);
    usleep(40000);
  }
}
