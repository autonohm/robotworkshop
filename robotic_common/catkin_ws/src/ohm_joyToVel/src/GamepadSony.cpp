/**
* @file   GamepadSony.cpp
* @author christian
* @date   05.12.2012
*/

#include "GamepadSony.h"

GamepadSony::GamepadSony(double vLin_max, double vAng_max, bool useVelocityPanTilt, Contour contour)
  : _contour_nr(contour),
    _saveInit(false),
    _vLin_max(vLin_max),
    _vAng_max(vAng_max)
{
  _panTiltCtrl.setMode(useVelocityPanTilt);
  _panTiltCtrl.setPanAnglesLimit(-1.0,    1.0);
  _panTiltCtrl.setTiltAnglesLimit(-0.43,  0.14);
  _panTiltCtrl.setHomePos(0.0, -0.55);
  _panTiltCtrl.setSpeedFactor(20.0);
}

void GamepadSony::rootVelocity(const sensor_msgs::Joy::ConstPtr& joy, geometry_msgs::TwistStamped& vel)
{
  double v=0.0, w=0.0;

  /* correction of joy message in this case. 
     For the game pad the value is 1 if the trigger is not pulled. 
     Pushed the trigger is going to -1. 
  */
  if (_saveInit)
  {
     double forward  = fabs(joy->axes[R2]);
     double backward = fabs(joy->axes[L2]);
     w = joy->axes[A1_Y];

     if (_contour_nr == LINEAR)
     {
       v = (forward - backward) * _vLin_max;
     }
     // _contour_nr == SQUARE
     else
     {
       v = ((forward * forward) - (backward * backward)) * _vLin_max;
       w = w * fabs(w) * _vAng_max;
     }

     vel.header.stamp    = joy->header.stamp;
     vel.twist.linear.x  = v;
     vel.twist.linear.y  = 0.0;
     vel.twist.linear.z  = 0.0;
     vel.twist.angular.x = 0.0;
     vel.twist.angular.y = 0.0;
     vel.twist.angular.z = w;
  }
  else
  {
     ROS_INFO_THROTTLE(5, "Please press forward and backward buttons for velocity at the same time to init joystick");
     if ((joy->buttons[B_R2] == 1) && (joy->buttons[B_L2] == 1))
     {
        _saveInit = true;
        ROS_INFO("Succesfully initialized");
     }
  }
}

void GamepadSony::rootButtons(const sensor_msgs::Joy::ConstPtr& joy, ohm_joyToVel::Joy_Action& joy_action)
{
  static ohm_joyToVel::Joy_Action joy_action_old;
  // publish on change
//  if (joy_action.status != joy_action_old.status)
  {
    if (joy->buttons[B_R1])
      joy_action.status  = joy_action.TYPE_AUTO_MAN;
    else if (joy->buttons[B_S])
      joy_action.status = joy_action.TYPE_CONFIRM;
    else if (joy->buttons[B_X])
      joy_action.status = joy_action.TYPE_CANCEL;
    else if (joy->buttons[B_START])
      joy_action.status = joy_action.TYPE_MENUE;
//    else if (joy->buttons[B_UP])
//      joy_action.status = joy_action.TYPE_LED_ON;
    else if (joy->buttons[B_RIGHT])
       joy_action.status = joy_action.TYPE_THERMO_CAM;
//    else if (joy->buttons[B_DOWN])
//       joy_action.status = joy_action.TYPE_BACK_CAM;
    else if (joy->buttons[B_LEFT])
       joy_action.status = joy_action.TYPE_KINECT_CAM;
    else if (joy->buttons[B_L1])
       joy_action.status = joy_action.TYPE_FIX_CAM;
    else if (joy->buttons[B_UP])
        joy_action.status = joy_action.TYPE_HOOK_TOGGLE_ACTION;
    else if (joy->buttons[B_DOWN])
        joy_action.status = joy_action.TYPE_HOOK_HOME;
    else
      joy_action.status = joy_action.TYPE_DEFAULT;

    if (joy_action.status == joy_action.TYPE_FIX_CAM)
    {
      _panTiltCtrl.homePos();
      _panTiltCtrl.lookBack();
    }
  }
  joy_action_old = joy_action;
}

void GamepadSony::rootCameraControl(const sensor_msgs::Joy::ConstPtr& joy, ohm_joyToVel::PanTiltCtrl& camCtrl)
{
//  std::cout << __PRETTY_FUNCTION__ << std::endl;


  // velocity controlled
  if(_panTiltCtrl.velUsed())
  {
    _panTiltCtrl.panTiltVel(-joy->axes[A2_Y]*fabs(joy->axes[A2_Y]), -joy->axes[A2_X]*fabs(joy->axes[A2_X]), camCtrl);
  }

  // position controlled
  else
  {
    _panTiltCtrl.panTiltPos(-joy->axes[A2_Y]*joy->axes[A2_Y], -joy->axes[A2_X]*joy->axes[A2_X], camCtrl);
  }
}

void GamepadSony::rootCameraControl(const sensor_msgs::Joy::ConstPtr& joy, ohm_joyToVel::SensorHeadCtrl& ctrl)
{
//  std::cout << __PRETTY_FUNCTION__ << std::endl;
  _panTiltCtrl.panTiltVel(joy->axes[A2_Y]*fabs(joy->axes[A2_Y]), joy->axes[A2_X]*fabs(joy->axes[A2_X]), ctrl);
}







