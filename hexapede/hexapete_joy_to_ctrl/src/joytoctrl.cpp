#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float64.h"

#include "joytoctrl.h"
#include "param.h"

JoyToCtrl::JoyToCtrl(void)
  : _prvnh("~"),
    _saveInit(false){

  _saveInit1 = _saveInit2 = _saveInit;

  _prvnh.param<int>("axis_linear", _linear, 1);
  _prvnh.param<int>("axis_angular", _angular, 0);
  _prvnh.param<int>("axis_pitch", _pitch, 2);
  _prvnh.param<int>("axis_yaw", _yaw ,3);

  std::cout << _linear << std::endl;
  std::cout << _angular << std::endl;
  std::cout << _pitch << std::endl;
  std::cout << _yaw << std::endl;

  _pub_pitch = _nh.advertise<std_msgs::Float64>("servo_pitch/command", 1);
  _pub_yaw   = _nh.advertise<std_msgs::Float64>("servo_yaw/command", 1);
  _pub_twist = _nh.advertise<geometry_msgs::Twist>("hexapete/cmd_vel", 1);
  _sub_joy   = _nh.subscribe<sensor_msgs::Joy>("joy", 10, &JoyToCtrl::_joyCallback, this);
}


void JoyToCtrl::_joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
  double _temp_x = 0;
  double _temp_z = 0;
  
  _temp_x = joy->axes[_linear];
  _temp_z = joy->axes[_angular];
  
  if((_temp_x < DEAD_X_P) && (_temp_x > DEAD_X_N)){
    _temp_x = 0;
  }
  if((_temp_z < DEAD_Z_P) && (_temp_z > DEAD_Z_N)){
    _temp_z = 0;
  }

  _msg_twist.linear.x  = _temp_x;
  _msg_twist.angular.z = _temp_z; 
  _msg_pitch.data = joy->axes[_pitch];
  _msg_yaw.data = joy->axes[_yaw];
}


void JoyToCtrl::publish(){
  if(!_saveInit){
    if(!_saveInit1){
      ROS_INFO_THROTTLE(5,"Please move throttle up and down to initialize Joystic!");
    }
    if(!_saveInit1 && _msg_twist.linear.x > 0){
      _saveInit1 = true;
      ROS_INFO("Positive throttle initialized!");
    }
    if(_saveInit1 && !_saveInit2 && _msg_twist.linear.x < 0){
      _saveInit2 = true;
      ROS_INFO("Negative throttle initialized!");
    }
    if(_saveInit1 && _saveInit2 && _msg_twist.linear.x == 0){
      _saveInit = true;
      ROS_INFO("Joystic successfully initialized!");
    }
    _msg_twist.linear.x  = 0;
    _msg_twist.angular.z = 0; 
    _msg_pitch.data = 0;
    _msg_yaw.data = 0;
  }
  _pub_twist.publish(_msg_twist);
  _pub_pitch.publish(_msg_pitch);
  _pub_yaw.publish(_msg_yaw);
}
