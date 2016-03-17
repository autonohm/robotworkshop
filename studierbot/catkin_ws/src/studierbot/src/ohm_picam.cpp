#include <ros/ros.h>
#include "PiCam.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ohm_picam_node");
    PiCam node;
    node.start(100);
}
