#ifndef MECHANUMSTEERING_H_
#define MECHANUMSTEERING_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include "../../drive/interface/Motorcontroller.h"

using namespace std;

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
  MechanumSteering(ChassisParams chParams, MotorParams* mParams);

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

  ros::NodeHandle _nh;
  ros::Subscriber _joySub;
  ros::Subscriber _velSub;

  ChassisParams    _chParams;
  MotorParams*     _mParams;
  Motorcontroller* _motor;

  // revolutions per minute for each channel (only 4 of 6 channels are used)
  float _rpm[6];

  // maximum velocity [m/s]
  float _vMax;

  // maximum rotating rate [rad]
  float _omegaMax;

  // conversion from [m/s] to revolutions per minute [RPM]
  float _ms2rpm;

  // conversion from revolutions per minute [RPM] to [m/s]
  float _rpm2ms;

  // distance of tracks
  float _track;

  // leverage for rotational movement, i.e. distance between kinematic center and wheel contact point
  float _leverage;

  // correction factor to address non-quadratic chassis
  float _tangentialFactor;

  // time elapsed since last call
  ros::Time _lastCmd;
};

#endif
