#include <ros/ros.h>

#include "../../skid_steering/src/SkidSteering.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "skid_steering_node");

  ChassisParams chParams;
  MotorParams   motorParams;

  // Assign motor channels to motor/wheel mounting
  ros::NodeHandle nh("~");

  nh.param("track",         chParams.track,           0.28f);
  nh.param("wheelBase",     chParams.wheelBase,       0.0f);
  nh.param("wheelDiameter", chParams.wheelDiameter,   0.06f);
  nh.param("chFrontLeft",   chParams.frontLeft,       0);
  nh.param("chFrontRight",  chParams.frontRight,      1);
  nh.param("chCenterLeft",  chParams.centerLeft,      2);
  nh.param("chCenterRight", chParams.centerRight,     3);
  nh.param("chRearLeft",    chParams.rearLeft,        4);
  nh.param("chRearRight",   chParams.rearRight,       5);
  nh.param("direction",     chParams.direction,       1);
  nh.param("canInterface",  motorParams.canInterface, std::string("slcan0"));
  nh.param("gearRatio",     motorParams.gearRatio,    131.f);
  nh.param("encoderRatio",  motorParams.encoderRatio, 64.f);
  nh.param("rpmMax",        motorParams.rpmMax,       80.f);
  nh.param("kp",            motorParams.kp,           1.f);
  nh.param("ki",            motorParams.ki,           0.f);
  nh.param("kd",            motorParams.kd,           0.f);
  nh.param("antiWindup",    motorParams.antiWindup,   1);

  cout << "Com Port: " << motorParams.comPort << endl;
  SkidSteering robot(chParams, motorParams);
  robot.run();
}
