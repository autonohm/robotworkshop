#include <iostream>

#include "../../mechanum_steering/src/MechanumSteering.h"
#include <std_msgs/Float32MultiArray.h>

using namespace std;

MechanumSteering::MechanumSteering(ChassisParams &chassisParams, MotorParams &motorParams, SocketCAN &can)
{
  _mc[0]    = new MotorControllerCAN(&can, 0, motorParams);
  _mc[1]    = new MotorControllerCAN(&can, 1, motorParams);

  _motorParams  = new MotorParams(motorParams);
  _chassisParams = chassisParams;
  // ensure that the direction parameter is set properly (either 1 or -1)
  if(_chassisParams.direction>0) _chassisParams.direction = 1;
  else _chassisParams.direction = -1;

  _rad2rpm          = (chassisParams.wheelBase+chassisParams.track)/chassisParams.wheelDiameter; // (lx+ly)/2 * 1/r
  _rpm2rad          = 1.0 / _rad2rpm;
  _ms2rpm           = 60.0/(chassisParams.wheelDiameter*M_PI);
  _rpm2ms           = 1.0 / _ms2rpm;
  _vMax             = motorParams.rpmMax * _rpm2ms;
  _omegaMax         = motorParams.rpmMax * _rpm2rad;

  _joySub = _nh.subscribe<sensor_msgs::Joy>("joy", 10, &MechanumSteering::joyCallback, this);
  _velSub = _nh.subscribe<geometry_msgs::Twist>("vel/teleop", 10, &MechanumSteering::velocityCallback, this);
  _pubRPM = _nh.advertise<std_msgs::Float32MultiArray>("rpm", 1);
  
  _rpm[0] = 0.0;
  _rpm[1] = 0.0;
  _rpm[2] = 0.0;
  _rpm[3] = 0.0;

  cout << "Initialized mechanum steering with vMax: " << _vMax << " m/s" << endl;
}

MechanumSteering::~MechanumSteering()
{
  delete _mc[0];
  delete _mc[1];
  delete _motorParams;
}

void MechanumSteering::run()
{
  ros::Rate rate(25);
  _lastCmd = ros::Time::now();
  unsigned int cnt;

  std_msgs::Float32MultiArray msgRPM;
  float rpm[4];
  
  bool run = true;
  while(run)
  {
    ros::spinOnce();

    ros::Duration dt = ros::Time::now() - _lastCmd;
    bool lag = (dt.toSec()>0.5);
    if(lag)
    {
      ROS_WARN_STREAM("Lag detected ... deactivate motor control");
      _mc[0]->stop();
      _mc[1]->stop();
    }
    else
    {
      if(cnt++>=25)
      {
        _mc[0]->broadcastExternalSync();
        cnt=0;
      }

      for(int i=0; i<=1; i++)
      {
        if(!_mc[i]->setRPM(&(_rpm[2*i])))
        {
          std::cout << "# Failed to set RPM values for CAN ID" << _mc[i]->getCanId() << std::endl;
        }
      }

      //_mc[0]->waitForSync();
      //_mc[1]->waitForSync();
      _mc[0]->getWheelResponse(rpm);
      _mc[1]->getWheelResponse(&(rpm[2]));
              
	   std::vector<float> vRPM;
	   vRPM.push_back(rpm[_chassisParams.frontLeft.id  * 2 + _chassisParams.frontLeft.channel]);
	   vRPM.push_back(rpm[_chassisParams.frontRight.id * 2 + _chassisParams.frontRight.channel]);
	   vRPM.push_back(rpm[_chassisParams.rearLeft.id   * 2 + _chassisParams.rearLeft.channel]);
	   vRPM.push_back(rpm[_chassisParams.rearRight.id  * 2 + _chassisParams.rearRight.channel]);
      msgRPM.data = vRPM;
      _pubRPM.publish(msgRPM);
    }

    run = ros::ok();

    rate.sleep();
  }

  _mc[0]->stop();
  _mc[1]->stop();
}

void MechanumSteering::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // Assignment of joystick axes to motor commands
  float fwd      = joy->axes[1];            // Range of values [-1:1]
  float left     = joy->axes[0];            // Range of values [-1:1]
  float turn     = joy->axes[2];            // Range of values [-1:1]
  float throttle = (joy->axes[3]+1.0)/2.0;  // Range of values [0:1]

  float vFwd  = throttle * fwd  * _vMax;
  float vLeft = throttle * left * _vMax;
  float omega = throttle * turn * _omegaMax;

  normalizeAndMap(vFwd, vLeft, omega);
}

void MechanumSteering::velocityCallback(const geometry_msgs::Twist::ConstPtr& cmd)
{
  normalizeAndMap(cmd->linear.x, cmd->linear.y, cmd->angular.z);
}

void MechanumSteering::normalizeAndMap(float vFwd, float vLeft, float omega)
{
  float rpmFwd   = vFwd  * _ms2rpm;
  float rpmLeft  = vLeft * _ms2rpm;
  float rpmOmega = omega * _rad2rpm;

  //cout << "vFwd: " << vFwd << "m/s, vLeft: " << vLeft << "m/s, omega: " << omega << endl;
  //cout << "rpmFwd: " << rpmFwd << ", rpmLeft: " << rpmLeft << ", rpmOmega: " << rpmOmega << endl;

  // leading signs -> see derivation: Stefan May, Skriptum Mobile Robotik
  _rpm[_chassisParams.frontLeft.id  * 2 + _chassisParams.frontLeft.channel]  =  rpmFwd - rpmLeft - rpmOmega;
  _rpm[_chassisParams.frontRight.id * 2 + _chassisParams.frontRight.channel] = -rpmFwd - rpmLeft - rpmOmega;
  _rpm[_chassisParams.rearLeft.id   * 2 + _chassisParams.rearLeft.channel]   =  rpmFwd + rpmLeft - rpmOmega;
  _rpm[_chassisParams.rearRight.id  * 2 + _chassisParams.rearRight.channel]  = -rpmFwd + rpmLeft - rpmOmega;

  // possibility to flip directions
  _rpm[0] *= _chassisParams.direction;
  _rpm[1] *= _chassisParams.direction;
  _rpm[2] *= _chassisParams.direction;
  _rpm[3] *= _chassisParams.direction;

  // Normalize values, if any value exceeds the maximum
  float rpmMax = std::abs(_rpm[0]);
  for(int i=1; i<4; i++)
  {
    if(std::abs(_rpm[i]) > _motorParams->rpmMax)
      rpmMax = std::abs(_rpm[i]);
  }
  if(rpmMax > _motorParams->rpmMax)
  {
    float factor = _motorParams->rpmMax / rpmMax;
    for(int i=0; i<4; i++)
      _rpm[i] *= factor;
  }

  _lastCmd = ros::Time::now();
}
