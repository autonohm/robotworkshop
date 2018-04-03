#include <iostream>

#include "../../skid_steering/src/SkidSteering.h"

using namespace std;

SkidSteering::SkidSteering(ChassisParams &chParams, MotorParams &params)
{

  _motor = new MotorControllerSerial(params);
  _chParams = chParams;
  _track                = _chParams.track;
  _pinionCircumference  = _chParams.wheelDiameter * M_PI;
  _vMax                 = _motor->getRPMMax() * _pinionCircumference / 60.f;

  _joySub = _nh.subscribe<sensor_msgs::Joy>(    "joy",        10, &SkidSteering::joyCallback,      this);
  _velSub = _nh.subscribe<geometry_msgs::Twist>("vel/teleop", 10, &SkidSteering::velocityCallback, this);

  _rpm[0] = 0.0;
  _rpm[1] = 0.0;
  _rpm[2] = 0.0;
  _rpm[3] = 0.0;
  _rpm[4] = 0.0;
  _rpm[5] = 0.0;
}

SkidSteering::~SkidSteering()
{
  delete _motor;
}

void SkidSteering::run()
{
  ros::Rate rate(25);
  _lastCmd = ros::Time::now();

  bool run = true;
  while(run)
  {
    ros::spinOnce();

    ros::Duration dt = ros::Time::now() - _lastCmd;
    bool lag = (dt.toSec()>0.5);
    if(lag)
    {
      ROS_WARN_STREAM("Lag detected ... deactivate motor control");
    }
    else
    {
      float rpmLeft  = trackspeedToRPM(_vl);
      float rpmRight = trackspeedToRPM(_vr);
      if(_chParams.frontLeft   >= 0)  _rpm[_chParams.frontLeft]   = rpmLeft;
      if(_chParams.centerLeft  >= 0)  _rpm[_chParams.centerLeft]  = rpmLeft;
      if(_chParams.rearLeft    >= 0)  _rpm[_chParams.rearLeft]    = rpmLeft;
      if(_chParams.frontRight  >= 0)  _rpm[_chParams.frontRight]  = rpmRight;
      if(_chParams.centerRight >= 0)  _rpm[_chParams.centerRight] = rpmRight;
      if(_chParams.rearRight   >= 0)  _rpm[_chParams.rearRight]   = rpmRight;

      //cout << _vl << " " << _vr << " " << _vMax << " " << rpmLeft << " " << rpmRight << endl;
      _motor->setRPM(_rpm);
      cout << _motor->getRPM(0) << " " << _motor->getRPM(1) << endl;
    }

    run = ros::ok();// && !lag;

    rate.sleep();
  }

  _motor->stop();
}

void SkidSteering::normalizeVelocity(float &vl, float &vr)
{
  float trackMax = abs(vl);
  if(abs(vr)>trackMax) trackMax=abs(vr);

  float scale = 1.f;
  if(trackMax>_vMax) scale = _vMax/trackMax;

  vl = scale * vl;
  vr = scale * vr;
}

void SkidSteering::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // Assignment of joystick axes to motor commands
  float linear  = joy->axes[1];
  float angular = joy->axes[2];
  float speed   = (joy->axes[3]+1.f)/2.f;

  _vl = _vMax * speed * (-linear+angular);
  _vr = _vMax * speed * (linear+angular);

  normalizeVelocity(_vl, _vr);

  _lastCmd = ros::Time::now();
}

void SkidSteering::velocityCallback(const geometry_msgs::Twist::ConstPtr& cmd)
{
  twistToTrackspeed(&_vl, &_vr, cmd->linear.x, cmd->angular.z);

  normalizeVelocity(_vl, _vr);

  _lastCmd = ros::Time::now();
}

void SkidSteering::twistToTrackspeed(float *vl, float *vr, float v, float omega) const
{
  *vr = -1.f * (v + omega * _track);
  *vl =       v - omega * _track;  
}

void SkidSteering::trackspeedToTwist(float vl, float vr, float *v, float *omega) const
{
  *v     = (vl + vr) / 2.f;
  *omega = (vr - vl) * _track;
}

float SkidSteering::trackspeedToRPM(float v) const
{
  return (v / _pinionCircumference * 60.f);
}
