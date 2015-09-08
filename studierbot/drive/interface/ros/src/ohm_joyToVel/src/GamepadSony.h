/**
* @file   GamepadSony.h
* @author Christian Pfitzner
* @date   06.12.2012
*/

#ifndef GAMEPAD_SONY_H_
#define GAMEPAD_SONY_H_

#include <ros/ros.h>
#include "ControllerBase.h"
#include "JoyPanTilt.h"

/**
 * @class   GamepadSony
 * Class derived from Interface ControllerBase
 */
class GamepadSony : public ControllerBase
{
public:
  /**
   * @enum Axis_ps3
   * axis of ps3 controller
   */
  enum Axis_ps3
  {
    A1_Y,    				  //!< y axis of left analog stick
    A1_X,    				  //!< x axis of left analog stick
    A2_Y,    				  //!< y axis of right analog stick
    A2_X,    				  //!< x axis of right analog stick
    CROSS_UP,				  //!< command cross up
    CROSS_R, 				  //!< command cross right
    CROSS_D, 				  //!< command cross down
    CROSS_L, 				  //!< command cross left
    L2=12,      				  //!< L2
    R2=13,      				  //!< R2
    L1,      				  //!< L1
    R1,      				  //!< R1
    AB_T,     				  //!< triangle button
    AB_C,     				  //!< circle button
    AB_X,     				  //!< cross button
    AB_S,      				//!< square botton
    SIXXAXIS_X,				//!< x axis of sixxaxis controller
    SIXXAXIS_Y,				//!< y axis of sixxaxis controller
    SIXAXXIS_Z
  };

  /**
   *  @enum Buttons_ps3
   *  Buttons of the ps3 controller
   */
  enum Buttons_ps3
  {
    B_SELECT,					//!< SELECT
    B_A1,    					//!< A1
    B_A2,    					//!< A2
    B_START, 					//!< START
    B_UP,    					//!< UP
    B_RIGHT, 					//!< RIGHT
    B_DOWN,  					//!< DOWN
    B_LEFT,  					//!< LEFT
    B_L2,                    //!< L2
    B_R2,                    //!< R2
    B_L1,                  //!< L1
    B_R1,                  //!< R1
    B_T,                 //!< triangle button
    B_C,                 //!< circle button
    B_X,                 //!< cross button
    B_S,                 //!< square botton
	};

    /**
     * Standard constructor
     */
    GamepadSony(double vLin_max, double vAng_max, bool useVelocityPanTilt = false, Contour contour = SQUARE);
    /**
     * Default destructor
     */
    ~GamepadSony(void) { }
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
    void rootCameraControl(const sensor_msgs::Joy::ConstPtr& joy, ohm_joyToVel::SensorHeadCtrl& ctrl);
    /**
     * Function to init joystick savely by pressing brake and forward at the same time
     * @param     joy         sensor message form joystick
     *
     * @note      This function is necessary for the joy ps3 driver: at the beginning the velocity axes
     *            have a value of 0. by pressing the value changes to 1 when pressed once.
     */
    void saveInit(const sensor_msgs::Joy::ConstPtr& joy);
  private:
    JoyPanTilt              _panTiltCtrl;
    ControllerBase::Contour _contour_nr;
    bool                    _saveInit;
    double                  _vLin_max;
    double                  _vAng_max;
};

#endif /* GAMEPAD_SONY_H_ */
