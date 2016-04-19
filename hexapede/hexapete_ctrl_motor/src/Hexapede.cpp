#include "../../../hexapede/hexapete_ctrl_motor/src/Hexapede.h"

#include <iostream>

#include "../../../hexapede/hexapete_ctrl_motor/src/params.h"
using namespace std;

Hexapede::Hexapede()
{
  _track               = TRACK;
  _wheelBase           = WHEELBASE;
  _gearRatio           = GEARRATIO;
  _wheelCircumference  = WHEELCICRUMFERENCE;
  _vMax                = _motor.getRPMMax() / _gearRatio * WHEELCICRUMFERENCE;

  _diagonal            = sqrt(_wheelBase*_wheelBase + _track*_track);
  _cosa                = cos(_track / _diagonal);

  ros::NodeHandle prvNh("~");
  prvNh.param<double>("lag_error", _lagTime, 1.0);

  _velSub = _nh.subscribe("hexapete/cmd_vel", 1, &Hexapede::velocityCallback, this);
}

void Hexapede::run()
{
  ros::Rate rate(30);
  _lastCmd = ros::Time::now();

  bool run = true;
  while(run)
  {
    ros::spinOnce();

    ros::Duration dt = ros::Time::now() - _lastCmd;
    bool lag = (dt.toSec() > _lagTime);
    if(lag)
    {
      //ROS_WARN_STREAM("Lag detected ... exiting robot control node");
      ROS_WARN_STREAM("Lag detected");
    }

    run = ros::ok(); //&& !lag;

    rate.sleep();
  }

  _motor.stop();
}

void Hexapede::velocityCallback(const geometry_msgs::Twist& cmd)
{
  double vl, vr;

  twistToTrackspeed(&vl, &vr, -cmd.linear.x, cmd.angular.z);

  cout << "vl: " << vl << ", vr: " << vr << endl;
  double rpm[6];
  enum eMotorChannel
  {
    channelLeftFront     = 0, //CH 1,2,3,4,5,6
    channelLeft          = 1, //CH ?
    channelLeftRear      = 2, //CH ?
    channelRightFront    = 3, //CH 1,2,3,4,5,6
    channelRight         = 4, //CH ?
    channelRightRear     = 5, //CH ?
  };


  //TEST
  rpm[channelLeft]       = trackspeedToTicksPerTurn(vl) * 60.0;
  rpm[channelLeftFront]  = rpm[channelLeft];
  rpm[channelLeftRear]   = rpm[channelLeft];
  rpm[channelRight]      = trackspeedToTicksPerTurn(vr) * 60.0;
  rpm[channelRightFront] = rpm[channelRight];
  rpm[channelRightRear]  = rpm[channelRight];

  _motor.setRPM(rpm);

  _lastCmd = ros::Time::now();
}

void Hexapede::twistToTrackspeed(double *vl, double *vr, double v, double omega) const
{
  *vr =  -1 * (v - omega * _diagonal / (2.0 * _cosa));
  *vl =       (v + omega * _diagonal / (2.0 * _cosa));
}

void Hexapede::trackspeedToTwist(double vl, double vr, double *v, double *omega) const
{
  *v     = (vl - vr) / 2.0;
  *omega = (vr + vl) * _cosa / (2.0 * _diagonal);
}

double Hexapede::trackspeedToTicksPerTurn(double v) const
{
  return (v / _wheelCircumference) * _gearRatio;
}
