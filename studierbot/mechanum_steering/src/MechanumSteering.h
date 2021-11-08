#ifndef MECHANUMSTEERING_H_
#define MECHANUMSTEERING_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include "MotorControllerCAN.h"

using namespace std;

struct WheelParams
{
  int id;      // ID of motor controller board, i.e., CAN ID
  int channel; // Channel of motor controller board, i.e., either 0 or 1
};

struct ChassisParams
{
  float track;
  float wheelBase;
  float wheelDiameter;
  struct WheelParams frontLeft;
  struct WheelParams frontRight;
  struct WheelParams centerLeft;
  struct WheelParams centerRight;
  struct WheelParams rearLeft;
  struct WheelParams rearRight;
  int   direction;

  ChassisParams()
  {
    track               = 0.f;
    wheelBase           = 0.f;
    wheelDiameter       = 0.f;
    frontLeft.id        = 0;
    frontLeft.channel   = 0;
    frontRight.id       = 0;
    frontRight.channel  = 0;
    rearLeft.id         = 0;
    rearLeft.channel    = 0;
    rearRight.id        = 0;
    rearRight.channel   = 0;
    direction           = 0;
  }
};

/**
 * @class Main class for robot drives based on mechanum steering
 * @author Stefan May
 * @date 08.10.2017
 */
class MechanumSteering
{
public:

  /**
   * Standard Constructor
   * @params[in] chassisParams chassis parameters, including the map for assigning channels to position of wheels
   * @params[in] motorParams motor parameters
   * @params[in] can socket can interface
   */
  MechanumSteering(ChassisParams &chassisParams, MotorParams &motorParams, SocketCAN &can);

  /**
   * Destructor
   */
  ~MechanumSteering();

  /**
   * ROS main loop (blocking method)
   */
  void run();

private:

  /**
   * ROS joystick callback
   * @param[in] joy message with joystick command
   */
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  /**
   * ROS command velocity callback
   * @param[in] cmd message with velocity command
   */
  void velocityCallback(const geometry_msgs::Twist::ConstPtr& cmd);

  /**
   * Normalize motion command and assign it to the channels
   * @param[in] vFwd forward velocity (x-axis)
   * @param[in] vLeft velocity to the left (y-axis)
   * @param[in] omega angular velocity (around z-axis)
   */
  void normalizeAndMap(float vFwd, float vLeft, float omega);

  ros::NodeHandle        _nh;
  ros::Subscriber        _joySub;
  ros::Subscriber        _velSub;
  ros::Publisher         _pubRPM;
  
  ChassisParams          _chassisParams;
  MotorParams*           _motorParams;
  MotorControllerCAN*    _mc[2];

  // revolutions per minute for each channel
  float                  _rpm[4];

  // maximum velocity [m/s]
  float                  _vMax;

  // maximum rotating rate [rad]
  float                  _omegaMax;

  // conversion from [m/s] to revolutions per minute [RPM]
  float                  _ms2rpm;

  // conversion from revolutions per minute [RPM] to [m/s]
  float                  _rpm2ms;

  // conversion from [rad/s] to revolutions per minute [RPM]
  float                  _rad2rpm;

  // conversion from revolutions per minute [RPM] to [rad/s]
  float                  _rpm2rad;

  // time elapsed since last call
  ros::Time              _lastCmd;
};

#endif
