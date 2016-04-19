#include <ros/ros.h>

#include "../../studierbot_base/src/PiCam.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ohm_picam_node");
    PiCam node;
    node.start(100);
}
