#include <ros/ros.h>
#include <ros/console.h>

#include "../../mechanum_steering/src/MechanumSteering.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mechanum_steering_node");
  ChassisParams chassisParams;
  MotorParams motorParams(Faulhaber_16002);

  // Assign motor channels to motor/wheel mounting
  ros::NodeHandle nh("~");

  nh.param("track",         chassisParams.track,         0.3f);
  nh.param("wheelBase",     chassisParams.wheelBase,     0.3f);
  nh.param("wheelDiameter", chassisParams.wheelDiameter, 0.1f);
  nh.param("chFrontLeft",   chassisParams.frontLeft,     0);
  nh.param("chFrontRight",  chassisParams.frontRight,    1);
  nh.param("chRearLeft",    chassisParams.rearLeft,      2);
  nh.param("chRearRight",   chassisParams.rearRight,     3);
  nh.param("direction",     chassisParams.direction,     1);
  nh.param("kp",            motorParams.kp,              1.f);
  nh.param("ki",            motorParams.ki,              0.f);
  nh.param("kd",            motorParams.kd,              0.f);
  nh.param("antiWindup",    motorParams.antiWindup,      1);

  MechanumSteering robot(chassisParams, &motorParams);
  robot.run();
}
