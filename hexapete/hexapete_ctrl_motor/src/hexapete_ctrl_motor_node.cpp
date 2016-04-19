#include "ros/ros.h"
#include "Hexapede.h"

using namespace std;

const char node[]= "hexapete_ctrl_motor_node";

int main(int argc, char **argv)
{
  ros::init(argc, argv, node);

  Hexapede robot;
  robot.run();

  return 0;
}
