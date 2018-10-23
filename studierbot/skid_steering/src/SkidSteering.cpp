#include <iostream>

#include "../../skid_steering/src/SkidSteering.h"

using namespace std;

SkidSteering::SkidSteering(ChassisParams &chassisParams, MotorParams &motorParams, SocketCAN &can)
{
  _mc[0]                = new MotorControllerCAN(&can, 0, motorParams);
  _mc[1]                = new MotorControllerCAN(&can, 1, motorParams);
  _mc[2]                = new MotorControllerCAN(&can, 2, motorParams);
  _chassisParams        = chassisParams;
  _track                = _chassisParams.track;
  _pinionCircumference  = _chassisParams.wheelDiameter * M_PI;
  _vMax                 = motorParams.rpmMax * _pinionCircumference / 60.f;

  _joySub = _nh.subscribe<sensor_msgs::Joy>(    "joy",        10, &SkidSteering::joyCallback,      this);
  _velSub = _nh.subscribe<geometry_msgs::Twist>("vel/teleop", 10, &SkidSteering::velocityCallback, this);

  _vl     = 0.0;
  _vr     = 0.0;
}

SkidSteering::~SkidSteering()
{
  delete _mc[0];
  delete _mc[1];
  delete _mc[2];
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

      float rpm[6] = {0.f};
      int fl = _chassisParams.frontLeft.id*2   + _chassisParams.frontLeft.channel;
      int fr = _chassisParams.frontRight.id*2  + _chassisParams.frontRight.channel;
      int cl = _chassisParams.centerLeft.id*2  + _chassisParams.centerLeft.channel;
      int cr = _chassisParams.centerRight.id*2 + _chassisParams.centerRight.channel;
      int rl = _chassisParams.rearLeft.id*2    + _chassisParams.rearLeft.channel;
      int rr = _chassisParams.rearRight.id*2   + _chassisParams.rearRight.channel;

      if(fl>=0 && fl<6) rpm[fl] = rpmLeft;
      if(fr>=0 && fr<6) rpm[fr] = rpmRight;
      if(cl>=0 && cl<6) rpm[cl] = rpmLeft;
      if(cr>=0 && cr<6) rpm[cr] = rpmRight;
      if(rl>=0 && rl<6) rpm[rl] = rpmLeft;
      if(rr>=0 && rr<6) rpm[rr] = rpmRight;

      for(int i=0; i<=2; i++)
      {
        if(!_mc[i]->setRPM(&(rpm[2*i])))
        {
          std::cout << "# Failed to set RPM values for CAN ID" << _mc[i]->getCanId() << std::endl;
        }
      }
    }

    run = ros::ok();// && !lag;

    rate.sleep();
  }

  _mc[0]->stop();
  _mc[1]->stop();
  _mc[2]->stop();
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
  *vl =         v - omega * _track;
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
