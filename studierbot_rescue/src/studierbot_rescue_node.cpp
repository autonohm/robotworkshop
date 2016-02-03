#include "ros/ros.h"
#include "StudierBotRescue.h"

using namespace std;

const char node[]= "studierbot_rescue_node";

int main(int argc, char **argv)
{
  ros::init(argc, argv, node);

  StudierBotRescue robot;
  robot.run();

  return 0;
}
