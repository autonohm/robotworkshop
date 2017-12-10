#include <ros/ros.h>
#include <ros/console.h>

#include "../../mechanum_steering/src/MechanumSteering.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mechanum_steering_node");
  MotorParams mParams(Faulhaber_16002);

  // Assign motor channels to motor/wheel mounting
  ros::NodeHandle nh("~");

  ChannelMap chMap;
  nh.param("chFrontLeft",  chMap.frontLeft,    0);
  nh.param("chFrontRight", chMap.frontRight,   1);
  nh.param("chRearLeft",   chMap.rearLeft,     2);
  nh.param("chRearRight",  chMap.rearRight,    3);
  nh.param("direction",    chMap.direction,    1);
  nh.param("kp",           mParams.kp,         1.f);
  nh.param("ki",           mParams.ki,         0.f);
  nh.param("kd",           mParams.kd,         0.f);
  nh.param("antiWindup",   mParams.antiWindup, 1);

  MechanumSteering robot(&mParams, chMap);
  robot.run();
}
