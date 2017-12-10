#include <ros/ros.h>

#include "../../skid_steering/src/SkidSteering.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "skid_steering_node");

  MotorParams mParams(Pololu_Gearmotor_37D);
  // Assign motor channels to motor/wheel mounting
  ros::NodeHandle nh("~");

  ChannelMap chMap;
  nh.param("chFrontLeft",   chMap.frontLeft,    0);
  nh.param("chFrontRight",  chMap.frontRight,   1);
  nh.param("chCenterLeft",  chMap.rearLeft,     2);
  nh.param("chCenterRight", chMap.rearRight,    3);
  nh.param("chRearLeft",    chMap.rearLeft,     4);
  nh.param("chRearRight",   chMap.rearRight,    5);
  nh.param("direction",     chMap.direction,    1);
  nh.param("kp",            mParams.kp,         1.f);
  nh.param("ki",            mParams.ki,         0.f);
  nh.param("kd",            mParams.kd,         0.f);
  nh.param("antiWindup",    mParams.antiWindup, 1);

  SkidSteering robot(mParams, chMap);
  robot.run();
}
