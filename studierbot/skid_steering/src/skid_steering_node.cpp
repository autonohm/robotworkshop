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

  nh.param("track",         chassisParams.track,               0.28f);
  nh.param("wheelBase",     chassisParams.wheelBase,           0.0f);
  nh.param("wheelDiameter", chassisParams.wheelDiameter,       0.06f);
  nh.param("idFrontLeft",   chassisParams.frontLeft.id,        0);
  nh.param("chFrontLeft",   chassisParams.frontLeft.channel,   0);
  nh.param("idFrontRight",  chassisParams.frontRight.id,       0);
  nh.param("chFrontRight",  chassisParams.frontRight.channel,  1);
  nh.param("idCenterLeft",  chassisParams.centerLeft.id,       1);
  nh.param("chCenterLeft",  chassisParams.centerLeft.channel,  0);
  nh.param("idCenterRight", chassisParams.centerRight.id,      1);
  nh.param("chCenterRight", chassisParams.centerRight.channel, 1);
  nh.param("idRearLeft",    chassisParams.rearLeft.id,         2);
  nh.param("chRearLeft",    chassisParams.rearLeft.channel,    0);
  nh.param("idRearRight",   chassisParams.rearRight.id,        2);
  nh.param("chRearRight",   chassisParams.rearRight.channel,   1);
  nh.param("direction",     chassisParams.direction,           1);
  nh.param("canInterface",  canInterface,                      std::string("slcan0"));
  nh.param("gearRatio",     motorParams.gearRatio,             131.f);
  nh.param("encoderRatio",  motorParams.encoderRatio,          64.f);
  nh.param("rpmMax",        motorParams.rpmMax,                80.f);
  nh.param("kp",            motorParams.kp,                    1.f);
  nh.param("ki",            motorParams.ki,                    0.f);
  nh.param("kd",            motorParams.kd,                    0.f);
  nh.param("antiWindup",    motorParams.antiWindup,            1);

  SocketCAN can(canInterface);

  cout << "CAN Interface: " << canInterface << endl;
  SkidSteering robot(chassisParams, motorParams, can);
  robot.run();
}
