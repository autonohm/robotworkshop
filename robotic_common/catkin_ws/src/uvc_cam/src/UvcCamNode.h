/*
 * UvcCamNode.h
 *
 *  Created on: 17.02.2014
 *      Author: phil
 */

#ifndef UVCCAMNODE_H_
#define UVCCAMNODE_H_

#include <ros/ros.h>

#include "obdevice/UvcCam.h"

#include <string>

class UvcCamNode
{
public:
  UvcCamNode();
  virtual ~UvcCamNode();
  void start(void);
private:
  void run(void);
  ros::NodeHandle _nh;
  ros::Publisher _imagePub;
  ros::Publisher _imageInfoPub;
  ros::Rate* _rate;
  obvious::UvcCam* _cam;
  unsigned char* _imgBuf;
  unsigned int _height;
  unsigned int _width;
  unsigned int _camFrameRate;
  std::string _camId;
  bool _compressed;
};

#endif /* UVCCAMNODE_H_ */
