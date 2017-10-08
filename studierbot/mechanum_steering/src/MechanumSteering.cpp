#include <iostream>

#include "../../mechanum_steering/src/MechanumSteering.h"
#include "../../mechanum_steering/src/params.h"

using namespace std;

MechanumSteering::MechanumSteering()
{
  _track                = TRACK;
  _pinionCircumference  = PINIONCIRCUMFERENCE;
  _vMax                 = _motor.getRPMMax() * PINIONCIRCUMFERENCE / 60.0;

  _joySub = _nh.subscribe<sensor_msgs::Joy>(    "joy",        10, &MechanumSteering::joyCallback,      this);
  _velSub = _nh.subscribe<geometry_msgs::Twist>("vel/teleop", 10, &MechanumSteering::velocityCallback, this);
}

void MechanumSteering::run()
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
      double rpmLeft  = trackspeedToRPM(_vl);
      double rpmRight = trackspeedToRPM(_vr);
      //cout << _vl << " " << _vr << " " << _vMax << " " << rpmLeft << " " << rpmRight << endl;
      _motor.setRPM(rpmLeft, rpmRight);
      cout << _motor.getRPM(0) << " " << _motor.getRPM(1) << endl;
    }

    run = ros::ok();// && !lag;

    rate.sleep();
  }

  _motor.stop();
}

void MechanumSteering::normalizeVelocity(double &vl, double &vr)
{
  double trackMax = abs(vl);
  if(abs(vr)>trackMax) trackMax=abs(vr);

  double scale = 1.0;
  if(trackMax>_vMax) scale = _vMax/trackMax;

  vl = scale * vl;
  vr = scale * vr;
}

void MechanumSteering::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // Assignment of joystick axes to motor commands
  double linear  = joy->axes[1];
  double angular = joy->axes[2];
  double speed   = (joy->axes[3]+1.0)/2.0;

  _vl = _vMax * speed * (-linear-angular);
  _vr = _vMax * speed * (linear-angular);

  normalizeVelocity(_vl, _vr);

  _lastCmd = ros::Time::now();
}

void MechanumSteering::velocityCallback(const geometry_msgs::Twist::ConstPtr& cmd)
{
  twistToTrackspeed(&_vl, &_vr, cmd->linear.x, cmd->angular.z);

  normalizeVelocity(_vl, _vr);

  _lastCmd = ros::Time::now();
}

void MechanumSteering::twistToTrackspeed(double *vl, double *vr, double v, double omega) const
{
  *vr = -1 * (v + omega * _track);
  *vl =       v - omega * _track;  
}

void MechanumSteering::trackspeedToTwist(double vl, double vr, double *v, double *omega) const
{
  *v     = (vl + vr) / 2.0;
  *omega = (vr - vl) * _track;
}

double MechanumSteering::trackspeedToRPM(double v) const
{
  return (v / _pinionCircumference * 60.0);
}
