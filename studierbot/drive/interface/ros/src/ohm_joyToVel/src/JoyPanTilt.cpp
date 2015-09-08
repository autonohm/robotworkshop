/**
* @file   JoyPanTilt.cpp
* @author Christian Pfitzner
* @date   10.06.2013
*
*
*/

#define PI 3.14159265359

#include "JoyPanTilt.h"
#include <ros/ros.h>

JoyPanTilt::JoyPanTilt(bool useVelCtrl)
: _useVelCtrl(useVelCtrl)
{
  _range.pan.min  = 0.0;
  _range.pan.max  = 0.0;
  _range.tilt.min = 0.0;
  _range.tilt.max = 0.0;
  _homePos.pan    = 0.0;
  _homePos.tilt   = 0.0;
  _oldAngle.pan   = 0.0;
  _oldAngle.tilt  = 0.0;
  _cameraFixed    = false;
  _debug = false;
}

void JoyPanTilt::setPanAnglesLimit(const double& min, const double& max)
{
  _range.pan.min = min * PI;
  _range.pan.max = max * PI;
  ROS_INFO("Panning range set successfully.");
}

void JoyPanTilt::setTiltAnglesLimit(const double& min, const double& max)
{
  _range.tilt.min = min * PI;
  _range.tilt.max = max * PI;
  ROS_INFO("Tilting range set successfully.");
}

void JoyPanTilt::setHomePos(const double& panAngle, const double& tiltAngle)
{
  if(this->velUsed())
  {
    _homePos.pan  = panAngle;
    _homePos.tilt = tiltAngle;
  }
  else
  {
    ROS_DEBUG("In position mode you can't reset to home position. Just release the joy stick");
  }
}

bool JoyPanTilt::lookBack(void)
{
  if(_debug)
    std::cout << __PRETTY_FUNCTION__ << std::endl;
  if(_useVelCtrl)
  {
    double now = ros::Time::now().toSec();
    static double lastTime;
    double timeDifference = now - lastTime;

    if (timeDifference < 0.3)
    {
      //std::cout << timeDifference << std::endl;
      _backPressed = true;
    }


    lastTime = now;
    return(true);
  }
  else
  {
    ROS_DEBUG("Can't look back because of position controlled joy stick mode");
    return(false);
  }
}

void JoyPanTilt::panTiltVel(const double& panValue, const double& tiltValue, ohm_joyToVel::PanTiltCtrl& camCtrl)
{
  if(_homingPressed)
  {
    // set POSITION
    _oldAngle.pan  = _homePos.pan;
    _oldAngle.tilt = _homePos.tilt;

    _homingPressed          = false;         // reset state
    camCtrl.speedControlled = false;
  }

  else if (_backPressed)
  {
    if(_debug)
      std::cout << __PRETTY_FUNCTION__ << std::endl;

    // set POSITION
    _oldAngle.tilt = _homePos.tilt;
    if(_oldAngle.pan > 0.0)
      _oldAngle.pan   = _range.pan.max;
    else if (_oldAngle.pan <= 0.0)
      _oldAngle.pan = _range.pan.min;

    _backPressed            = false;        // reset state
    camCtrl.speedControlled = false;
  }
  else
  {
    // limit angles for pan and tilt
    this->limitAngles(_oldAngle.pan,  _range.pan.min,  _range.pan.max);
    this->limitAngles(_oldAngle.tilt, _range.tilt.min, _range.tilt.max);

    // set VELOCITY
    _oldAngle.pan   =  panValue  * _speed;
    _oldAngle.tilt  =  tiltValue * _speed;
    camCtrl.speedControlled = true;
  }

  // switched sign for mathematical correct rotation
  camCtrl.viewAngleLeft = -_oldAngle.pan;
  camCtrl.viewAngleUP   = -_oldAngle.tilt;


}

void JoyPanTilt::panTiltPos(const double& panValue, const double& tiltValue, ohm_joyToVel::PanTiltCtrl& camCtrl)
{
  if(!_cameraFixed)
  {
     _oldAngle.pan  = camCtrl.viewAngleLeft = panValue  * PI;
     _oldAngle.tilt = camCtrl.viewAngleUP   = tiltValue * PI;

     // limit angles for pan and tilt
     this->limitAngles(_oldAngle.pan, _range.pan.min, _range.pan.max);
     this->limitAngles(_oldAngle.tilt, _range.tilt.min, _range.tilt.max);
  }
  else
  {
     camCtrl.viewAngleLeft   = _oldAngle.pan;
     camCtrl.viewAngleUP     = _oldAngle.tilt;
  }
  camCtrl.speedControlled = false;
}

void JoyPanTilt::panTiltVel(const double& panValue, const double& tiltValue, ohm_joyToVel::SensorHeadCtrl& ctrl)
{
  if(_homingPressed)
  {
    // set POSITION
    ctrl.pan.pos   = _homePos.pan;
    ctrl.tilt.pos = _homePos.tilt;

    _homingPressed          = false;         // reset state
  }

  else if (_backPressed)
  {
    if(_debug)
      std::cout << __PRETTY_FUNCTION__ << std::endl;

    // set POSITION
    ctrl.tilt.pos = _homePos.tilt;
    if(_oldAngle.pan > 0.0)
      ctrl.pan.pos   = _range.pan.max;
    else if (_oldAngle.pan <= 0.0)
      ctrl.pan.pos = _range.pan.min;
  }
  else
  {
    // limit angles for pan and tilt
    this->limitAngles(_oldAngle.pan,  _range.pan.min,  _range.pan.max);
    this->limitAngles(_oldAngle.tilt, _range.tilt.min, _range.tilt.max);

    // set VELOCITY
    ctrl.pan.vel   =  panValue  * _speed;
    ctrl.tilt.vel  =  tiltValue * _speed;
  }
//
//  // switched sign for mathematical correct rotation
//  ctrl.pan.vel;
//  ctrl.tilt.vel;
}

void JoyPanTilt::panTiltPos(const double& panValue, const double& tiltValue, ohm_joyToVel::SensorHeadCtrl& ctrl)
{
 /* 
  if(!_cameraFixed)
  {
     _oldAngle.pan  = ctrl.viewAngleLeft = panValue  * PI;
     _oldAngle.tilt = ctrl.viewAngleUP   = tiltValue * PI;

     // limit angles for pan and tilt
     this->limitAngles(_oldAngle.pan, _range.pan.min, _range.pan.max);
     this->limitAngles(_oldAngle.tilt, _range.tilt.min, _range.tilt.max);
  }
  else
  {
     ctrl.viewAngleLeft   = _oldAngle.pan;
     ctrl.viewAngleUP     = _oldAngle.tilt;
  }
  ctrl.speedControlled = false;*/
}

bool JoyPanTilt::homePos(void)
{
  if(_useVelCtrl)
  {
    _homingPressed = true;
    return(true);
  }
  else
  {
    ROS_INFO("Can't set command for homing position, because of position mode.");
    return(false);
  }

}

bool JoyPanTilt::fixPos(bool button)
{
  if(_debug)
    std::cout << __PRETTY_FUNCTION__ << std::endl;
  // check mode
  if(_useVelCtrl)
  {
    ROS_INFO("Can't set command for fixing position, because of velocity mode.");
    return(false);
  }
  else
  {
    static bool oldStatus;
    bool edge = button && !oldStatus;

    if (edge && !_cameraFixed)
       _cameraFixed = true;
    else if (edge && _cameraFixed)
       _cameraFixed = false;
    oldStatus = button;
    return(true);
  }
}

void JoyPanTilt::limitAngles(double& angle, const double& minTH, const double& maxTH)
{
  if((angle >= maxTH) || (angle < minTH))
  {
    if (angle > maxTH)
      angle = maxTH;
    else if (angle < minTH)
      angle = minTH;
  }
}
