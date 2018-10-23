#include <ros/ros.h>

#include "../../skid_steering/src/SkidSteering.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "skid_steering_node");

  ChassisParams chassisParams;
  MotorParams   motorParams;

  // Assign motor channels to motor/wheel mounting
  ros::NodeHandle nh("~");

  std::string canInterface;

  nh.param("track",         chassisParams.track,           0.28f);
  nh.param("wheelBase",     chassisParams.wheelBase,       0.0f);
  nh.param("wheelDiameter", chassisParams.wheelDiameter,   0.06f);
  nh.param("chFrontLeft",   chassisParams.frontLeft,       0);
  nh.param("chFrontRight",  chassisParams.frontRight,      1);
  nh.param("chCenterLeft",  chassisParams.centerLeft,      2);
  nh.param("chCenterRight", chassisParams.centerRight,     3);
  nh.param("chRearLeft",    chassisParams.rearLeft,        4);
  nh.param("chRearRight",   chassisParams.rearRight,       5);
  nh.param("direction",     chassisParams.direction,       1);
  nh.param("canInterface",  canInterface,                  std::string("slcan0"));
  nh.param("canID",         motorParams.canID,             0);
  nh.param("gearRatio",     motorParams.gearRatio,         131.f);
  nh.param("encoderRatio",  motorParams.encoderRatio,      64.f);
  nh.param("rpmMax",        motorParams.rpmMax,            80.f);
  nh.param("kp",            motorParams.kp,                1.f);
  nh.param("ki",            motorParams.ki,                0.f);
  nh.param("kd",            motorParams.kd,                0.f);
  nh.param("antiWindup",    motorParams.antiWindup,        1);

  SocketCAN can(canInterface);

  cout << "CAN Interface: " << canInterface << endl;
  SkidSteering robot(chassisParams, motorParams);
  robot.run();
}
