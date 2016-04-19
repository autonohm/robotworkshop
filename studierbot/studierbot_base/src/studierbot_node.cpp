#include <ros/ros.h>

#include "../../studierbot_base/src/StudierBot.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "studierbot_node");
  StudierBot robot;
  robot.run();
}
