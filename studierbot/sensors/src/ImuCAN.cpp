#include "ImuCAN.h"
#include <iostream>
#include <cmath>
#include <string.h>
#include <iomanip>
#include <unistd.h>

#define GROUPID    (0x02 << 9)
#define SYSTEMID   (0x5  << 6)
#define COMPINPUT  (0x0  << 5)
#define COMPOUTPUT (0x1  << 5)

ImuCAN::ImuCAN(SocketCAN* can, bool verbosity)
{
  _q.resize(4);

  _can       = can;
  _cf.can_id = GROUPID | SYSTEMID | COMPINPUT | 0x0;

  canid_t canidOutput = GROUPID | SYSTEMID | COMPOUTPUT | 0x0;

  _pubPoseGyro        = _n.advertise<geometry_msgs::PoseStamped>("poseGyro", 1);

  setCANId(canidOutput);
  can->registerObserver(this);

  usleep(25000);
}

ImuCAN::~ImuCAN()
{

}

void ImuCAN::notify(struct can_frame* frame)
{
  //canid_t id0 = GROUPID | SYSTEMID | COMPOUTPUT | 0x0;
  if(frame->can_dlc==8)
  {
    short* sdata = (short*)(frame->data);
    _q[0] = ((float)sdata[0]) / 20000.f;
    _q[1] = ((float)sdata[1]) / 20000.f;
    _q[2] = ((float)sdata[2]) / 20000.f;
    _q[3] = ((float)sdata[3]) / 20000.f;

    // Publish IMU message
    // Publish pure gyroscope estimation
    geometry_msgs::PoseStamped msgPoseGyro;
    msgPoseGyro.header.frame_id    = "map";
    msgPoseGyro.header.stamp       = ros::Time::now();
    msgPoseGyro.pose.position.x    = 0;
    msgPoseGyro.pose.position.y    = 0;
    msgPoseGyro.pose.position.z    = 0;
    msgPoseGyro.pose.orientation.x = _q[1];
    msgPoseGyro.pose.orientation.y = _q[2];
    msgPoseGyro.pose.orientation.z = _q[3];
    msgPoseGyro.pose.orientation.w = _q[0];

    _pubPoseGyro.publish(msgPoseGyro);
  }
}
