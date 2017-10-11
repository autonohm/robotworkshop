#ifndef MECHANUMSTEERING_H_
#define MECHANUMSTEERING_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include "../../mechanum_steering/src/Motorcontroller.h"

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
   */
  MechanumSteering();

  /**
   * ROS main loop (blocking method)
   */
  void run();

private:

  /**
   * ROS joystick callback
   * @param joy message with joystick command
   */
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  /**
   * ROS command velocity callback
   * @param cmd message with velocity command
   */
  void velocityCallback(const geometry_msgs::Twist::ConstPtr& cmd);

  void normalizeAndMap(double vFwd, double vLeft, double omega);

  ros::NodeHandle _nh;
  ros::Subscriber _joySub;
  ros::Subscriber _velSub;

  Motorcontroller _motor;

  double _rpm[6];

  // maximum velocity [m/s]
  double _vMax;

  // maximum rotating rate [rad]
  double _omegaMax;

  // conversion from [m/s] to revolutions per minute [RPM]
  double _ms2rpm;

  // conversion from revolutions per minute [RPM] to [m/s]
  double _rpm2ms;

  // distance of tracks
  double _track;

  // leverage for rotational movement, i.e. distance between kinematic center and wheel contact point
  double _leverage;

  // correction factor to address non-quadratic chassis
  double _tangentialFactor;

  // time elapsed since last call
  ros::Time _lastCmd;
};

#endif
