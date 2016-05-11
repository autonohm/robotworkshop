#include "../../../hexapede/hexapede_ctrl_motor/src/Hexapede.h"
#include "ros/ros.h"

using namespace std;

const char node[]= "hexapete_ctrl_motor_node";

int main(int argc, char **argv)
{
  ros::init(argc, argv, node);

  Hexapede robot;
  robot.run();

  return 0;
}
