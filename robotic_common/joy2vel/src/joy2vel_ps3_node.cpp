 /*
   joy2vel_ps3_node.cpp

   This programm is an easy to understand ros node to convert incoming
   joy messages to twist messages

   Copyright (C)  2015 Christian Pfitzner

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA

*/

// includes for ros
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

// global variables
ros::Subscriber g_joy_sub;
ros::Publisher  g_twist_pub;
bool            g_isInitialized = false;



/**
 * @enum JOY_AXIS_MAPPING
 */
enum JOY_AXIS_MAPPING{
   FORWARD  = 13,          // this axis is the R2 padel button at the ps3 game pad
   BACKWARD = 12,          // this axis is the L2 padel button at the ps3 game pad
   TURN     = 0            // this is the left analoge joystick
};


/**
 * Callback function to for joy messages
 * @param[in]     msg         message with type sensor_msgs::Joy
 */
void joyCallback(const sensor_msgs::Joy& msg)
{
   /*
    * Due to a bug in the joy driver, the signum of the axis values
    * might change after pressing the an axis after startup of the node.
    * To avoid any misbehavior it is necessary to initialize the node
    * by pressing forward an backwards together at the same time
    */
   if(msg.axes[FORWARD] == 1.0 && msg.axes[BACKWARD] == 1.0)
      g_isInitialized = true;


   if(g_isInitialized)
   {
      // generate twist message from joy msg;
      geometry_msgs::Twist twist;
      twist.linear.x  = (-msg.axes[FORWARD] + msg.axes[BACKWARD]) * 0.5;
      twist.angular.z = msg.axes[TURN];

      // comment this in for debugging
//      ROS_INFO_STREAM("linear x: " << twist.linear.x << " angular z: " << twist.angular.z);

      g_twist_pub.publish(twist);
   }
   else
   {
      ROS_WARN_STREAM_ONCE("Please press R2 and L2 at PS3 controller together for initialization");
   }
}





int main(int argc, char **argv)
{
   /*
    * Initialize ros node with name joy2vel_ps3_node
    */
   ros::init(argc, argv, "joy2vel_ps3_node");

   /*
    * Set up ros node handle. It is the main access point to communicate with the ROS system
    */
   ros::NodeHandle n;

   /*
    * Initialize subscriber for joy message. The subscriber is listening
    * for messages with the topic name <joy>. If a message is received,
    * the joyCallback function is called.
    */
   g_joy_sub   = n.subscribe("joy", 1, joyCallback);

   /*
    * Initialize publisher for twist messages to control the robot. The message
    * published will have the topic name <cmd_vel> and will be of the type
    * geometry_msgs::Twist
    */
   g_twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

   /*
    * ros::spin() will enter a loop for callbacks.
    * The loop will exit if the function ros::ok() returns false, e.g. Ctrl+C is pressed
    */
   ros::spin();
}
