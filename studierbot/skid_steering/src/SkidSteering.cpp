#include <iostream>

#include "../../skid_steering/src/SkidSteering.h"

using namespace std;

SkidSteering::SkidSteering(ChassisParams &chassisParams, MotorParams &motorParams, SocketCAN &can)
{
  _mc[0]                = new MotorControllerCAN(&can, 0, motorParams, true);
  _mc[1]                = new MotorControllerCAN(&can, 1, motorParams, true);
  _mc[2]                = new MotorControllerCAN(&can, 2, motorParams, true);
  _chassisParams        = chassisParams;
  _track                = _chassisParams.track;
  _pinionCircumference  = _chassisParams.wheelDiameter * M_PI;
  _vMax                 = motorParams.rpmMax * _pinionCircumference / 60.f;

  _joySub = _nh.subscribe<sensor_msgs::Joy>(    "joy",        10, &SkidSteering::joyCallback,      this);
  _velSub = _nh.subscribe<geometry_msgs::Twist>("vel/teleop", 10, &SkidSteering::velocityCallback, this);

  _vl          = 0.f;
  _vr          = 0.f;
  _fireButton  = 0;
  _cooliehatLR = 0.f;

  _addon = new AddonShieldCAN(&can);
  float servoFrequency = 330.f;
  _addon->setPWMFrequency(2, (int)servoFrequency);
  float uCenter = 1500.f;
  int centerPoint = (int)(uCenter / ((1.f / servoFrequency) * 1000000.f) * 100.f);
  cout << "centerPoint: " << centerPoint << endl;
  _addon->enable(2);
  _addon->enable(3);
  _addon->setPulseWidth(2, centerPoint);
  _addon->setPulseWidth(3, 30);
}

SkidSteering::~SkidSteering()
{
  delete _mc[0];
  delete _mc[1];
  delete _mc[2];

  _addon->disable(2);
  _addon->disable(3);
  delete _addon;
}

void SkidSteering::run()
{
  ros::Rate rate(25);
  _lastCmd = ros::Time::now();
  unsigned int cnt = 0;
  bool run = true;
  bool frontWheelsOnly;
  bool rearWheelsOnly;

  float uSetpointLowPass = 1500.f;
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

      if(cnt++>=25)
      {
        _mc[0]->broadcastExternalSync();
        cnt=0;
      }

      for(int i=0; i<=2; i++)
      {
        if(!_mc[i]->setRPM(&(rpm[2*i])))
        {
          std::cout << "# Failed to set RPM values for CAN ID" << _mc[i]->getCanId() << std::endl;
        }
      }
//Joystick button mapping section
      if(_thumbButton)
      {
        //Invertierung
      }
      if(_bottomButton7)
      {
        if(!frontWheelsOnly)
        {
         _mc[1]->disable();
         _mc[2]->disable();
        } else
        {
          _mc[1]->enable();
          _mc[2]->enable();
        }
      }

      if(_bottomButton8)
      {
        if(!rearWheelsOnly)
        {
         _mc[0]->disable();
         _mc[1]->disable();
        } else
        {
          _mc[0]->enable();
          _mc[1]->enable();
        }
      }

      float servoFrequency = 330.f;
      float uSetpoint = 1500.f + (_cooliehatLR) * 500.f;
      uSetpointLowPass = 0.9f * uSetpointLowPass + 0.1f * uSetpoint;
      int pulseWidth = (int)(uSetpointLowPass / ((1.f / servoFrequency) * 1000000.f) * 100.f);
      _addon->setPulseWidth(2, pulseWidth);
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

  //Define button vars
  _fireButton  = joy->buttons[0];
  _thumbButton = joy->buttons[1];
  _ulButton = joy->buttons[4];
  _urButton = joy->buttons[5];
  _llButton = joy->buttons[2];
  _lrButton = joy->buttons[3];
  _bottomButton7 = joy->buttons[6];
  _bottomButton8 = joy->buttons[7];
  _bottomButton9 = joy->buttons[8];
  _bottomButton10 = joy->buttons[9];
  _buttonButton11 = joy->buttons[10];
  _buttonButton12 = joy->buttons[11];
  _cooliehatLR = (float)joy->axes[4];

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
