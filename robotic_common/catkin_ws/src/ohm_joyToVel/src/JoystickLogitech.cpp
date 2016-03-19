/**
* @file JoystickLogitech.cpp
* @autor christian
* @date  05.12.2012
*
*
*/

#include "JoystickLogitech.h"

void JoystickLogitech::rootVelocity(const sensor_msgs::Joy::ConstPtr& joy, geometry_msgs::TwistStamped& vel)
{
  //variables
  double r=0.0, v=0.0, w=0.0;
  float th=0.0;

  // axis
  th = VMAX *((joy->axes[THROTTLE] + 1.0) / 2.0);

  if(fabs(th*joy->axes[PITCH]) > 0.02)
     v = th * joy->axes[PITCH];
  else
    v=0.0;

  if(fabs(th*joy->axes[ROLL]) > 0.02)
    w = th * joy->axes[ROLL];
  else
    w=0.0;

  // rotation
  if(!((joy->axes[COOLI_Y]>0.4)||(joy->axes[COOLI_Y]<-0.4)) &&
      ((joy->axes[COOLI_X]>0.4)||(joy->axes[COOLI_X]<-0.4)))     //if cooliehat  pushed->no rotaional command
    r = 0.0;
  else
  {
    if(fabs(th*joy->axes[YAW]) > 0.02)
      r = th * joy->axes[YAW];
    else
      r=0.0;
  }

  ///@todo check if threshold is neccessary
  // threshold
  if(fabs(r)>10e-6 && fabs(v)<10e-3 && fabs(w)<10e-3)
    v = 0.0; w = r;

  vel.header.stamp    = joy->header.stamp;
  vel.twist.linear.x  = v;
  vel.twist.linear.y  = 0.0;
  vel.twist.linear.z  = 0.0;
  vel.twist.angular.x = 0.0;
  vel.twist.angular.y = 0.0;
  vel.twist.angular.z = w;

}

void JoystickLogitech::rootButtons(const sensor_msgs::Joy::ConstPtr& joy, ohm_joyToVel::Joy_Action& joy_action)
{
  static ohm_joyToVel::Joy_Action joy_action_old;
  // publish on change
  if (joy_action.status != joy_action_old.status)
  {
    if (joy->buttons[12-1])
       joy_action.status  = joy_action.TYPE_AUTO_MAN;
    else if (joy->buttons[2-1])
      joy_action.status   = joy_action.TYPE_DISABLE_TELE_ASS;
    else if (joy->buttons[5-1])
      joy_action.status   = joy_action.TYPE_CONFIRM;
    else if (joy->buttons[3-2])
      joy_action.status   = joy_action.TYPE_CANCEL;
    else if (joy->buttons[11-1])
      joy_action.status   = joy_action.TYPE_RETURN_HOME;
    else if (joy->buttons[1-1])
      joy_action.status   = joy_action.TYPE_FOCUS_CAM;
    else
      joy_action.status = 0;
  }
}

void JoystickLogitech::rootCameraControl(const sensor_msgs::Joy::ConstPtr& joy, ohm_joyToVel::PanTiltCtrl& camCtrl)
{
   camCtrl.viewAngleLeft = joy->axes[COOLI_Y];
   camCtrl.viewAngleUP   = joy->axes[COOLI_X];
}




