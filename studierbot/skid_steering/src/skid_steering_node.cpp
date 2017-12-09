#include <ros/ros.h>

#include "../../skid_steering/src/SkidSteering.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "skid_steering_node");
  MotorParams params(Pololu_Gearmotor_37D);
  SkidSteering robot(params);
  robot.run();
}
