/**
* @file   JoystickLogitech.h
* @author Christian Pfitzner
* @date   06.12.2012
*/

#ifndef __JOYSTICK_LOGITECH_H_
#define __JOYSTICK_LOGITECH_H_

#include "ControllerBase.h"

/**
 * @class JoystickLogitech
 */
class JoystickLogitech : public ControllerBase
{
public:
    /**
     * @enum Axis_joy
     * axis of joystick
     */
    enum Axis_joy
    {
      ROLL, 				//!< y_Axis
      PITCH, 				//!< x_Axis
      YAW,				//!< z_Axis for rotation
      THROTTLE,				//!< throttle poti of joystick
      COOLI_Y, 				//!< cooliehead x-axiss
      COOLI_X				//!< cooliehead y-axis
    };
    /**
     * Default constructor
     */
    JoystickLogitech(void)  { }
    /**
     * Function to root the velocity by specified axis
     * @param     joy   sensor message from joystick
     * @param     vel   velocity message
     */
    void rootVelocity(const sensor_msgs::Joy::ConstPtr& joy, geometry_msgs::TwistStamped& vel);
    /**
     * Function to root buttons by specified buttons on controller
     * @param     joy         sensor message form joystick
     * @param     joy_action  joy action message
     */
    void rootButtons (const sensor_msgs::Joy::ConstPtr& joy, ohm_joyToVel::Joy_Action& joy_action);
    /**
     * Function to root joystick axis to camera specified axis
     * @param     joy         sensor message from joystick
     * @param     camCtrl     message for camera control in yaw and pitch
     */
    void rootCameraControl(const sensor_msgs::Joy::ConstPtr& joy, ohm_joyToVel::PanTiltCtrl& camCtrl);

    void rootCameraControl(const sensor_msgs::Joy::ConstPtr& joy, ohm_joyToVel::SensorHeadCtrl& ctrl)
    {

    }
};


#endif /* JOYSTICK_LOGITECH_H_ */
