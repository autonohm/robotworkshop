#include <ros/ros.h>

#include "../../skid_steering/src/SkidSteering.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "skid_steering_node");

  ChassisParams chParams;
  MotorParams mParams(Pololu_Gearmotor_37D);

  // Assign motor channels to motor/wheel mounting
  ros::NodeHandle nh("~");

  nh.param("track",         chParams.track,         0.28f);
  nh.param("wheelBase",     chParams.wheelBase,     0.0f);
  nh.param("wheelDiameter", chParams.wheelDiameter, 0.06f);
  nh.param("chFrontLeft",   chParams.frontLeft,        0);
  nh.param("chFrontRight",  chParams.frontRight,       1);
  nh.param("chCenterLeft",  chParams.rearLeft,         2);
  nh.param("chCenterRight", chParams.rearRight,        3);
  nh.param("chRearLeft",    chParams.rearLeft,         4);
  nh.param("chRearRight",   chParams.rearRight,        5);
  nh.param("direction",     chParams.direction,        1);
  nh.param("kp",            mParams.kp,             1.f);
  nh.param("ki",            mParams.ki,             0.f);
  nh.param("kd",            mParams.kd,             0.f);
  nh.param("antiWindup",    mParams.antiWindup,     1);

  SkidSteering robot(chParams, mParams);
  robot.run();
}
