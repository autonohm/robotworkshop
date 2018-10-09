#ifndef MECHANUMSTEERING_H_
#define MECHANUMSTEERING_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include "../../drive/interface/MotorControllerCAN.h"

using namespace std;

struct ChassisParams
{
  float track;
  float wheelBase;
  float wheelDiameter;
  int   frontLeft;
  int   frontRight;
  int   centerLeft;
  int   centerRight;
  int   rearLeft;
  int   rearRight;
  int   direction;

  ChassisParams()
  {
    track         = 0.f;
    wheelBase     = 0.f;
    wheelDiameter = 0.f;
    frontLeft     = 0;
    frontRight    = 0;
    centerLeft    = 0;
    centerRight   = 0;
    rearLeft      = 0;
    rearRight     = 0;
    direction     = 0;
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
   * @params[in] chParams chassis parameters, including the map for assigning channels to position of wheels
   * @params[in] mParams motor parameters
   */
  MechanumSteering(ChassisParams &chParams, MotorParams &mParams);

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

  ChassisParams          _chParams;
  MotorParams*           _mParams;
  MotorControllerCAN*    _motor;

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
