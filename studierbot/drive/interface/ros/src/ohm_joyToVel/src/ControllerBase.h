/*
 * controller.h
 *
 *  Created on: 21.09.2012
 *      Author: chris
 */

#ifndef __CONTROLLER_H_
#define __CONTROLLER_H_

#include <sensor_msgs/Joy.h>
#include "ohm_joyToVel/Joy_Action.h"
#include "ohm_joyToVel/PanTiltCtrl.h"
#include "ohm_joyToVel/SensorHeadCtrl.h"
#include <geometry_msgs/TwistStamped.h>
#include <math.h>

/**
 * @def VMAX 0.7 meter per secound
 */
#define VMAX 1.3

/**
 * @class ControllerBase
 * Interface class for all kind of controllers
 */
class ControllerBase
{
	public:
    /**
     * @enum Contour
     */
    enum Contour
    {
      LINEAR = 1,       //!< linear contour of driving speed
      SQUARE            //!< square contour of driving speed
    };
    /**
     * Virtual destructor for class
     */
    virtual ~ControllerBase(void) { }
    /**
     * Virtual Function to root buttons for different controllers
     * @param[in]   joy           joy message from ros
     * @param[out]  joy_action    joy_action message from ohm_joyToVel
     */
    virtual void rootButtons (const sensor_msgs::Joy::ConstPtr& joy, ohm_joyToVel::Joy_Action& joy_action) = 0;
    /**
     * Virtual function to root velocity for different controllers
     * @param[in]   joy           joy message from ros
     * @param[out]  vel           velocity message
     */
    virtual void rootVelocity(const sensor_msgs::Joy::ConstPtr& joy, geometry_msgs::TwistStamped& vel) = 0;
    /**
     * Function to root joystick axis to camera specified axis
     * @param     joy         sensor message from joystick
     * @param     camCtrl     message for camera control in yaw and pitch
     */
    virtual void rootCameraControl(const sensor_msgs::Joy::ConstPtr& joy, ohm_joyToVel::PanTiltCtrl& camCtrl) = 0;

    virtual void rootCameraControl(const sensor_msgs::Joy::ConstPtr& joy, ohm_joyToVel::SensorHeadCtrl& ctrl) = 0;
};

#endif /* __CONTROLLER_H_ */
