#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"

class JoyToCtrl{
public:
  JoyToCtrl();
  void publish();

private:
  void _joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle _prvnh;
  ros::NodeHandle _nh;
  ros::Publisher  _pub_twist;
  ros::Publisher  _pub_pitch;
  ros::Publisher  _pub_yaw;
  ros::Subscriber _sub_joy;

  int _linear, _angular;
  int _pitch, _yaw;
  bool _saveInit, _saveInit1, _saveInit2;

  //msgs to send
  geometry_msgs::Twist _msg_twist;
  std_msgs::Float64    _msg_pitch;
  std_msgs::Float64    _msg_yaw;
};
