#include "JoyToVelNode.h"

#include <string>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <std_msgs/Float64.h>

using namespace std;
using namespace ros;


JoyToVelNode* JoyToVelNode::getInstance(void)
{
   if (!_instance)  _instance = new JoyToVelNode();
   return _instance;
}


JoyToVelNode::~JoyToVelNode(void)
{
   delete _ctrl;

   if(_instance != 0) {
      delete _instance;
      _instance = 0;
   }
}


void JoyToVelNode::run(void)
{
   ros::Rate r(_looprate);
   // ros loop
   while(ros::ok())
   {
     ros::spinOnce();
     if (_joy_init) this->publishPanTiltCmd(_joy);

     r.sleep();
   }
   ros::shutdown();
}


JoyToVelNode::JoyToVelNode(void)
{
   ros::NodeHandle private_nh("~");

   double vLin_max = 0.0;
   double vAng_max = 0.0;

   // Parameters for launch file
   private_nh.param("joy_topic",            _joy_topic,            std::string("joy"));
   private_nh.param("twist_topic",          _twist_topic,          std::string("vel/twist"));
   private_nh.param("joy_publisher_topic",  _joy_action_pub_topic, std::string("joy/action"));
   private_nh.param("vel_publisher_topic",  _vel_pub_topic,        std::string("vel/teleop"));
   private_nh.param("cam_publisher_topic",  _panTilt_topic,        std::string("panTilt/teleop"));
   private_nh.param("sensor_head",          _sensor_head_topic,    std::string("ohm_sensor_head_control"));
   private_nh.param("driving_contour",      _contour,              std::string("linear"));
   private_nh.param("change_namepsace",     _change_ns_topic,      std::string("change_namespace"));
   private_nh.param("useVelocityPanTilt",   _useVelocityPanTilt,   bool(true));
   private_nh.param("joystickUsed",         _joystickUsed,         bool(false));
   private_nh.param("debug",                _debug,                bool(false));
   private_nh.param <double>("loop_rate",   _looprate,             30.0);
   private_nh.param<double>("v_lin_max",    vLin_max,              1.0);
   private_nh.param<double>("v_ang_max",    vAng_max,              1.0);

   // Publishers
   _vel_pub        = _nh.advertise<geometry_msgs::TwistStamped>( _vel_pub_topic,        1);
   _tw_pub         = _nh.advertise<geometry_msgs::Twist>(        _twist_topic,          1);
   _joy_action_pub = _nh.advertise<ohm_joyToVel::Joy_Action>(    _joy_action_pub_topic, 1);
   _sensorHead_pub = _nh.advertise<ohm_joyToVel::SensorHeadCtrl>(_sensor_head_topic,    1);

   std::cout << "VelPublisher: " << _vel_pub.getTopic() << std::endl;
   std::cout << "JoyActionPub: " << _joy_action_pub.getTopic() << std::endl;
//   std::cout << "Sensor"

   // Subscribers
   _joy_sub = _nh.subscribe(_joy_topic, 20, &JoyToVelNode::joyCallback, this);

   // Service client
   _ns_srv = _nh.advertiseService(_change_ns_topic, &JoyToVelNode::namespaceCallback, this);

   // Initialisation
   _error_count   = 0;
   _contour_nr    = 0;
   _controller_nr = 0;
   _joy_init      = false;




   // Contour
   if      (strcmp(_contour.c_str(), "linear") == 0)  _contour_nr = ControllerBase::LINEAR;
   else if (strcmp(_contour.c_str(), "square") == 0 ) _contour_nr = ControllerBase::SQUARE;
   else
   {
      ROS_ERROR("Unknown driving contour. Please have a look at the launch file. Allowed contour: linear, square");
      _error_count++;
   }

   if (_joystickUsed) _ctrl = new JoystickLogitech;
   else               _ctrl = new GamepadSony(vLin_max, vAng_max, _useVelocityPanTilt, (ControllerBase::Contour)_contour_nr);

   // shutdown ros node if error occured in start up
   if (_error_count == 0) ROS_INFO("OHM_joyToVel startet succesful.");
   else
   {
      ROS_ERROR("OHM_joyToVel will be shutdown, because %d errors occurred", _error_count);
      ros::shutdown();
   }
}

void JoyToVelNode::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  _joy      = joy;
  _joy_init = true;
  this->publishVelocityCmd(joy);
  this->publishSensorHeadCmd(joy);
  this->publishPanTiltCmd(joy);
  this->publishJoyAction(joy);
}


void JoyToVelNode::publishVelocityCmd(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::TwistStamped *vel = new geometry_msgs::TwistStamped;   //toDo: Dellocation? Memory Leak? new without delete
  geometry_msgs::Twist twist;
  _ctrl->rootVelocity(joy, *vel);
  twist = vel->twist;
  _vel_pub.publish(*vel);
  _tw_pub.publish(twist);
}

void JoyToVelNode::publishPanTiltCmd(const sensor_msgs::Joy::ConstPtr& joy)
{
  ohm_joyToVel::PanTiltCtrl  cam_action_pub;
  static  ohm_joyToVel::PanTiltCtrl cam_action_pub_old;
  static bool stop, publishedStop;
  _ctrl->rootCameraControl(joy, cam_action_pub);

  if((cam_action_pub.viewAngleLeft == 0.0) && (cam_action_pub.viewAngleUP == 0.0) && (cam_action_pub.speedControlled == true)) {
    stop = true;
  }
  if((cam_action_pub.viewAngleLeft != 0.0) || (cam_action_pub.viewAngleUP != 0.0) || (cam_action_pub.speedControlled == false)) {
    stop = false;
  }

  // save new to old
  cam_action_pub_old = cam_action_pub;
}

void JoyToVelNode::publishSensorHeadCmd(const sensor_msgs::JoyConstPtr& Joy)
{
  ohm_joyToVel::SensorHeadCtrl ctrl;
  static ohm_joyToVel::SensorHeadCtrl ctrl_old;
  _ctrl->rootCameraControl(Joy, ctrl);
  _sensorHead_pub.publish(ctrl);

  ctrl_old = ctrl;
}

void JoyToVelNode::publishJoyAction(const sensor_msgs::Joy::ConstPtr& joy)
{
  ohm_joyToVel::Joy_Action    *joy_action_pub  = new ohm_joyToVel::Joy_Action;
  _ctrl->rootButtons(joy,*joy_action_pub);
  _joy_action_pub.publish(*joy_action_pub);
}

bool JoyToVelNode::namespaceCallback(ohm_joyToVel::ChangeNamespace::Request     &req,
                                         ohm_joyToVel::ChangeNamespace::Response     &res)
{
  std::string name_space= req.ns;

  // shut down
  _joy_sub.shutdown();
  _vel_pub.shutdown();
  _joy_action_pub.shutdown();
  _sensorHead_pan_pub.shutdown();

  // reconfigure namspaces
  _vel_pub        = _nh.advertise<geometry_msgs::TwistStamped>( name_space + _vel_pub_topic       ,     1);
  _joy_action_pub = _nh.advertise<ohm_joyToVel::Joy_Action>(    name_space + _joy_action_pub_topic,     1);
  _sensorHead_pub = _nh.advertise<ohm_joyToVel::SensorHeadCtrl>(name_space + _sensor_head_topic   ,     1);
  _joy_sub        = _nh.subscribe(_joy_topic, 20, &JoyToVelNode::joyCallback, this);  //toDo: correct? Topic was fixed before -> phil

  std::cout << "Changed to topic: " << _vel_pub.getTopic() << std::endl;

  return(true);
}

/*
 * Main program
 */
int main(int argc,char **argv)
{
   ros::init(argc, argv, "JoyToVel_Node");
   JoyToVelNode::getInstance()->run();
}



