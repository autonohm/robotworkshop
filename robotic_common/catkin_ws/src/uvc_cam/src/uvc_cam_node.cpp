/*
 * main.cpp
 *
 *  Created on: 17.02.2014
 *      Author: phil
 */

#include "UvcCamNode.h"

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "uvc_cam_node");
  UvcCamNode camNode;
  camNode.start();
}


