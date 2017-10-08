#include <ros/ros.h>

#include "../../mechanum_steering/src/MechanumSteering.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mechanum_steering_node");
  MechanumSteering robot;
  robot.run();
}
