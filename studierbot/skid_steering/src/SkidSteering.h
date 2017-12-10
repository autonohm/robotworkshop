#ifndef SKIDSTEERING_H_
#define SKIDSTEERING_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include "../../drive/interface/Motorcontroller.h"

using namespace std;

/**
 * @class Main class for robot drives based on skid steering
 * @author Stefan May
 * @date 15.05.2014
 */
class SkidSteering
{
public:

  /**
   * Standard Constructor
   * @param[in] params motor parameters
   */
  SkidSteering(MotorParams params, ChannelMap map);

  /**
   * Destructor
   */
  ~SkidSteering();

  /**
   * ROS main loop (blocking method)
   */
  void run();

private:

  /**
   * Normalize velocity, i.e., scale values, if one of both exceeds vMax
   */
  void normalizeVelocity(float &vl, float &vr);

  /**
   * 2D Motion model: computes tracks velocity based on linear and angular velocity
   */
  void twistToTrackspeed(float *vl, float *vr, float v, float omega) const;

  /**
   * 2D Motion model: computes linear and angular velocity based on tracks velocity
   */
  void trackspeedToTwist(float vl, float vr, float *v, float *omega) const;

  /**
   * Translates the metric speed v into the needed motor RPM
   */
  float trackspeedToRPM(float v) const;

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

  ros::NodeHandle _nh;
  ros::Subscriber _joySub;
  ros::Subscriber _velSub;

  Motorcontroller* _motor;
  ChannelMap       _channelMap;

  // revolutions per minute for each channel (only 2 of 6 channels are used)
  float _rpm[6];

  float _vl, _vr;

  // maximum velocity [m/s]
  float _vMax;

  // distance of tracks
  float _track;

  // circumference of pinion
  float _pinionCircumference;

  // time elapsed since last call
  ros::Time _lastCmd;
};

#endif
