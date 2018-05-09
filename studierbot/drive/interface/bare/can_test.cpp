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
  MotorParams params;
  MotorControllerCAN mc(params);
  mc.enable();

  for(char i=0; i<0x7F; i++)
  {
    mc.setPWM(i);
    usleep(40000);
  }
  for(char i=0x7F; i>-0x7F; i--)
  {
    mc.setPWM(i);
    usleep(40000);
  }
  for(char i=-0x7F; i<=0x00; i++)
  {
    mc.setPWM(i);
    usleep(40000);
  }
}
