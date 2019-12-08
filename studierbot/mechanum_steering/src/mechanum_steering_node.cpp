#include <ros/ros.h>
#include <ros/console.h>

#include "../../mechanum_steering/src/MechanumSteering.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mechanum_steering_node");
  ChassisParams chassisParams;
  MotorParams motorParams;

  // Assign motor channels to motor/wheel mounting
  ros::NodeHandle nh("~");

  std::string canInterface;
  int maxPulseWidth;
  int frequencyScale;
  nh.param("track",          chassisParams.track,               0.3f);
  nh.param("wheelBase",      chassisParams.wheelBase,           0.3f);
  nh.param("wheelDiameter",  chassisParams.wheelDiameter,       0.1f);
  nh.param("idFrontLeft",    chassisParams.frontLeft.id,        0);
  nh.param("chFrontLeft",    chassisParams.frontLeft.channel,   0);
  nh.param("idFrontRight",   chassisParams.frontRight.id,       0);
  nh.param("chFrontRight",   chassisParams.frontRight.channel,  1);
  nh.param("idRearLeft",     chassisParams.rearLeft.id,         1);
  nh.param("chRearLeft",     chassisParams.rearLeft.channel,    0);
  nh.param("idRearRight",    chassisParams.rearRight.id,        1);
  nh.param("chRearRight",    chassisParams.rearRight.channel,   1);
  nh.param("direction",      chassisParams.direction,           1);
  nh.param("canInterface",   canInterface,                      std::string("slcan0"));
  nh.param("frequencyScale", frequencyScale,                    32);
  nh.param("gearRatio",      motorParams.gearRatio,             131.f);
  nh.param("encoderRatio",   motorParams.encoderRatio,          64.f);
  nh.param("rpmMax",         motorParams.rpmMax,                80.f);
  nh.param("maxPulseWidth",  maxPulseWidth,                     63);
  nh.param("kp",             motorParams.kp,                    1.f);
  nh.param("ki",             motorParams.ki,                    0.f);
  nh.param("kd",             motorParams.kd,                    0.f);
  nh.param("antiWindup",     motorParams.antiWindup,            1);
  nh.param("invertEnc",      motorParams.invertEnc,             0);

  if(maxPulseWidth<0)   maxPulseWidth = 0;
  if(maxPulseWidth>127) maxPulseWidth = 127;
  motorParams.maxPulseWidth = (unsigned char)maxPulseWidth;

  if(frequencyScale<1)   frequencyScale = 1;
  if(frequencyScale>100) frequencyScale = 100;
  motorParams.frequencyScale = (unsigned short)frequencyScale;

  SocketCAN can(canInterface);
  cout << "CAN Interface: " << canInterface << endl;

  MechanumSteering robot(chassisParams, motorParams, can);
  robot.run();
}
