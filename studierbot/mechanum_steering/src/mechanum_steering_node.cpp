#include <ros/ros.h>

#include "../../mechanum_steering/src/MechanumSteering.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mechanum_steering_node");
  MotorParams params(Faulhaber_16002);
  ChannelMap map;
  map.frontLeft  = 0;
  map.frontRight = 2;
  map.rearLeft   = 1;
  map.rearRight  = 3;
  map.direction  = -1;
  MechanumSteering robot(&params, map);
  robot.run();
}
