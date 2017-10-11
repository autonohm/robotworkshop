#include <iostream>

#include "../../mechanum_steering/src/MechanumSteering.h"
#include "../../mechanum_steering/src/params.h"

using namespace std;

MechanumSteering::MechanumSteering()
{
  _leverage         = sqrt(WHEELBASE*WHEELBASE+TRACK*TRACK)/2.0;
  _tangentialFactor = 1.0/cos(atan2(WHEELBASE, TRACK)-(M_PI/4.0));
  _ms2rpm           = 60.0/(WHEELDIAMETER*M_PI);
  _rpm2ms           = 1.0 / _ms2rpm;
  _vMax             = _motor.getRPMMax() * _rpm2ms * sin(M_PI/4.0);
  _omegaMax         = _vMax / (_tangentialFactor * _leverage);

  _joySub = _nh.subscribe<sensor_msgs::Joy>("joy", 10, &MechanumSteering::joyCallback, this);
  _velSub = _nh.subscribe<geometry_msgs::Twist>("vel/teleop", 10, &MechanumSteering::velocityCallback, this);

  _rpm[0] = 0.0;
  _rpm[1] = 0.0;
  _rpm[2] = 0.0;
  _rpm[3] = 0.0;
  _rpm[4] = 0.0;
  _rpm[5] = 0.0;

  cout << "Initialized mechanum steering with vMax: " << _vMax << " m/s" << endl;
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
      _motor.stop();
    }
    else
    {
      _motor.setRPM(_rpm);
    }

    run = ros::ok();// && !lag;

    rate.sleep();
  }

  _motor.stop();
}

void MechanumSteering::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // Assignment of joystick axes to motor commands
  double fwd   = joy->axes[1];            // Range of values [-1:1]
  double left  = joy->axes[0];            // Range of values [-1:1]
  double turn  = joy->axes[2];            // Range of values [-1:1]
  double speed = (joy->axes[3]+1.0)/2.0;  // Range of values [0:1]

  double vFwd  = speed * fwd  * _vMax;
  double vLeft = speed * left * _vMax;
  double omega = speed * turn * _omegaMax;

  normalizeAndMap(vFwd, vLeft, omega);
}

void MechanumSteering::velocityCallback(const geometry_msgs::Twist::ConstPtr& cmd)
{
  normalizeAndMap(cmd->linear.x, cmd->linear.y, cmd->angular.z);
}

void MechanumSteering::normalizeAndMap(double vFwd, double vLeft, double omega)
{
  double rpmFwd   = vFwd  / sin(M_PI/4.0) * _ms2rpm;
  double rpmLeft  = vLeft / sin(M_PI/4.0) * _ms2rpm;
  double rpmOmega = omega / sin(M_PI/4.0) * _tangentialFactor * _leverage * _ms2rpm;

  //cout << "vFwd: " << vFwd << "m/s, vLeft: " << vLeft << "m/s, omega: " << omega << endl;
  //cout << "rpmFwd: " << rpmFwd << ", rpmLeft: " << rpmLeft << ", rpmOmega: " << rpmOmega << endl;

  _rpm[0] = rpmFwd - rpmLeft - rpmOmega; // front left
  _rpm[1] = rpmFwd + rpmLeft - rpmOmega; // rear left
  _rpm[2] = rpmFwd + rpmLeft + rpmOmega; // front right
  _rpm[3] = rpmFwd - rpmLeft + rpmOmega; // rear right

  // remap direction due to motor mounting (flip direction of left side)
  _rpm[0] = -_rpm[0];
  _rpm[1] = -_rpm[1];

  // Normalize values, if any value exceeds the maximum
  double rpmMax = std::abs(_rpm[0]);
  for(int i=1; i<4; i++)
  {
    if(std::abs(_rpm[i]) > RPMMAX)
      rpmMax = std::abs(_rpm[i]);
  }
  if(rpmMax > RPMMAX)
  {
    double factor = RPMMAX / rpmMax;
    for(int i=0; i<4; i++)
      _rpm[i] *= factor;
  }

  _lastCmd = ros::Time::now();
}
