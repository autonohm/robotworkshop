/**
 * @brief 		Class to generate twist message out of joy message
 *
 * @author 		Christian Pfitzner
 * @date 		2012-08-28
 * @version		0.1

 */

#ifndef __JOYTOVEL__
#define __JOYTOVEL__

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <geometry_msgs/Twist.h>

#include "ControllerBase.h"
#include "GamepadSony.h"
#include "JoystickLogitech.h"

#include "ohm_joyToVel/ChangeNamespace.h"

/**
 * @class 		JoyToVelNode
 * @author    Christian Pfitzner
 */
class JoyToVelNode
{
public:
  static JoyToVelNode* getInstance(void);
  /**
   * Standard destructor
   */
  ~JoyToVelNode(void);
  /**
   * Main loop of ROS Node
   */
  void run(void);
  /**
   * Function to set looprate
   * @param   rate in loops per second
   */
  void setLooprate(const double rate)     { _looprate = rate; }
  /**
   * Function to return looprate
   * @return
   */
  const double& getLooprate(void) const   { return _looprate; }
private:
  /**
   * Constructor of DiagnoseNode
   */
   JoyToVelNode(void);
   /**
    * Overloaded default copy constructor
    * @param
    */
   JoyToVelNode ( const JoyToVelNode& ) { }
    /**
    * Callback function for joy topic
    * @param[in]      joy         joystick message
    */
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    static JoyToVelNode*  _instance;      //!< Instance of ros node

    void publishVelocityCmd(const sensor_msgs::Joy::ConstPtr& joy);
    void publishPanTiltCmd(const sensor_msgs::Joy::ConstPtr& joy);
    void publishSensorHeadCmd(const sensor_msgs::JoyConstPtr& Joy);
    void publishJoyAction(const sensor_msgs::Joy::ConstPtr& joy);

    bool namespaceCallback(ohm_joyToVel::ChangeNamespace::Request     &req,
                            ohm_joyToVel::ChangeNamespace::Response    &res);


    ros::NodeHandle     _nh;		          //!< Ros node handle
    ControllerBase*     _ctrl;

    /**
    * Subscribers
    */
    ros::Subscriber _joy_sub; 		        //!< topic to subscriber joy messages
    /**
    * Publishers
    */
    ros::Publisher _vel_pub; 			        //!< topic to publish velocity messages
    ros::Publisher _tw_pub;               //!< topic for not stamped twist toDO: Remove stamped
    ros::Publisher _joy_action_pub;		    //!< topic to publish ohm normed buttons
    ros::Publisher _panTilt_pub;          //!< publisher for pan tilt messages
    ros::Publisher _arm_pub;              //!< publisher for robot arm messages
    ros::Publisher _sensorHead_pub;
    ros::Publisher _sensorHead_pan_pub;
    ros::Publisher    _sensorHead_tilt_pub;
    /**
     * Services
     */
    ros::ServiceServer _ns_srv;

    geometry_msgs::Twist vel;			        //!< twist topic for publisher
    sensor_msgs::Joy::ConstPtr _joy;

    unsigned int _controller_nr;		      //!< idx number for controller
    unsigned int _contour_nr; 			      //!< idx number for driving contour
    unsigned int _error_count; 			    //!< counter for errors in constuctor

    bool         _joy_init;

    /**
    * Parameters for launch file
    */
    std::string    _joy_topic; 			      //!< topic of joystick message
    std::string    _twist_topic;          //!< topic for twist message
    std::string    _joy_action_pub_topic; //!< topic of joystick publisher
    std::string    _vel_pub_topic; 		    //!< topic of velocity publisher
    std::string    _panTilt_topic;        //!< topic for pan tilt unit to control
    std::string    _arm_topic;            //!< topic for robot arm to control
    std::string    _sensor_head_topic;    //!< topic for sensor head toDO: Label correct? -> phil
    std::string    _change_ns_topic;      //!< topic for change namespace service


    double         _looprate; 			        //!< loop rate for ros node
    std::string    _contour; 			          //!< name of driving contour
    bool           _joystickUsed;           //!< TRUE if logitech device should be used, FALSE for PS3 Controller
    bool           _useVelocityPanTilt;    //!< TRUE to set pan tilt with velocity
    bool           _debug; 			            //!< TRUE for debuging
};

JoyToVelNode* JoyToVelNode::_instance = 0;

#endif  // __JOYTOVEL__



