/**
 * @author Stefan May
 * @date 06.08.2019
 * @brief Test program for CAN interface of IMU board
 */
#include <unistd.h>
#include <iostream>
#include <cmath>
#include "ImuCAN.h"

using namespace std;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "can_imu_node");

  SocketCAN can(std::string("slcan0"));
  can.startListener();

  ImuCAN imu(&can, true);
  usleep(2000000);

  while(ros::ok())
  {
    usleep(10000);
  }

  can.stopListener();

}
