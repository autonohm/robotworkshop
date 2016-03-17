#include <ros/ros.h>

#include "StudierBot.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "studierbot_node");
  StudierBot robot;
  robot.run();
}
