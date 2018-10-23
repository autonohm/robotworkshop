#ifndef SKIDSTEERING_H_
#define SKIDSTEERING_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include "../../drive/interface/MotorControllerCAN.h"

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
  WheelParams frontLeft;
  WheelParams frontRight;
  WheelParams centerLeft;
  WheelParams centerRight;
  WheelParams rearLeft;
  WheelParams rearRight;
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
    centerLeft.id       = 0;
    centerLeft.channel  = 0;
    centerRight.id      = 0;
    centerRight.channel = 0;
    rearLeft.id         = 0;
    rearLeft.channel    = 0;
    rearRight.id        = 0;
    rearRight.channel   = 0;
    direction           = 0;
  }
};

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
   * @param[in] chassisParams chassis parameters, including the map for assigning channels to position of wheels
   * @param[in] motorParams motor parameters
   * @param[in] can socket CAN instance
   */
  SkidSteering(ChassisParams &chassisParams, MotorParams &motorParams, SocketCAN &can);

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

  ros::NodeHandle        _nh;
  ros::Subscriber        _joySub;
  ros::Subscriber        _velSub;

  ChassisParams          _chassisParams;
  MotorControllerCAN*    _mc[3];

  float                  _vl, _vr;

  // maximum velocity [m/s]
  float                  _vMax;

  // distance of tracks
  float                  _track;

  // circumference of pinion
  float                  _pinionCircumference;

  // time elapsed since last call
  ros::Time              _lastCmd;
};

#endif
