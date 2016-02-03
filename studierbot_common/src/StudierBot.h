#ifndef STUDIERBOT_H_
#define STUDIERBOT_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>

#include "Motorcontroller.h"

#include <iostream>

using namespace std;

/**
 * @class Main class for low-cost robot platform
 * @author Stefan May
 * @date 30.05.2014
 */
class StudierBot
{
public:

  /**
   * Standard Constructor
   */
  StudierBot();

  /**
   * ROS main loop (blocking method)
   */
  void run();

private:

  /**
   * 2D Motion model: computes tracks velocity based on linear and angular velocity
   */
  void twistToTrackspeed(double *vl, double *vr, double v, double omega) const;

  /**
   * 2D Motion model: computes linear and angular velocity based on tracks velocity
   */
  void trackspeedToTwist(double vl, double vr, double *v, double *omega) const;

  /**
   * Translates the metric speed v into the needed motor RPM
   */
  double trackspeedToTicksPerTurn(double v) const;

  /**
   * ROS joystick callback
   * @param joy message with joystick command
   */
  //void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  /**
   * ROS command velocity callback
   * @param cmd message with velocity command
   */
  void velocityCallback(const geometry_msgs::TwistStamped& cmd);

  ros::NodeHandle _nh;
  //ros::Subscriber _joySub;
  ros::Subscriber _velSub;

  Motorcontroller _motor;

  // maximum velocity [m/s]
  double _vMax;

  // distance of wheels (axis length)
  double _track;

  // distance of axes
  double _wheelBase;

  // ratio between wheel revolution and motor revolution
  double _gearRatio;

  // circumference of wheels
  double _wheelCircumference;

  // diagonal of robot base (wheel contact)
  double _diagonal;

  // cosine of alpha from kinematic center to front right wheel
  double _cosa;

  // time elapsed since last call
  ros::Time _lastCmd;
};

#endif
