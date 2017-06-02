#include <ros/ros.h>

#include "../../tank_chassis/src/TankChassis.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tankchassis_node");
  TankChassis robot;
  robot.run();
}
