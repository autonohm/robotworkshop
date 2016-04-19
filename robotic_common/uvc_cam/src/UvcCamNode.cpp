/*
 * UvcCamNode.cpp
 *
 *  Created on: 17.02.2014
 *      Author: phil
 */

#include "UvcCamNode.h"

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <string>

#define EST_HEADER 420 // Estimated size of MJPEG header
#define MAX_WIDTH 1920
#define MAX_HEIGHT 1080


UvcCamNode::UvcCamNode()
{
  ros::NodeHandle prvNh("~");
  std::string strVar;
  double rateVar = 0.0;
  int width = 0;
  int height = 0;
  int camFrameRate = 0;
  prvNh.param<int>("cam_width", width, 640);
  prvNh.param<int>("cam_height", height, 480);
  prvNh.param<int>("cam_frame_rate", camFrameRate, 30);
  prvNh.param<bool>("compressed", _compressed, false);
  _width = static_cast<unsigned int>(width);
  _height = static_cast<unsigned int>(height);
  _camFrameRate = static_cast<unsigned int>(camFrameRate);
  prvNh.param<double>("rate", rateVar, 10);
  _rate = new ros::Rate(rateVar);
  prvNh.param("image_topic", strVar, std::string("UvcCamNode/image"));
  if(_compressed)
    _imagePub = _nh.advertise<sensor_msgs::CompressedImage>(strVar +  "/compressed", 1);
  else
    _imagePub = _nh.advertise<sensor_msgs::Image>(strVar, 1);
  prvNh.param("cam_serial", strVar, std::string("4EFC4C0F"));
  prvNh.param("cam_id", _camId, std::string("2"));   //0 driver 1 right 2 left toDo: Change that
  char* path = NULL;
  obvious::UvcCam::FindDevice(strVar.c_str(), path);
  if(!path)
  {
    std::cout << __PRETTY_FUNCTION__ << " error! Device " << strVar << " could not be found!\n";
    exit(1);
  }
  std::cout << " path = " << path << "\n";
  _cam = new obvious::UvcCam(path, _width, _height);
  _imgBuf = new unsigned char[MAX_WIDTH * MAX_HEIGHT * 3];
  prvNh.param("cam_info_topic", strVar, std::string("UvcCamNode/image_info"));
  _imageInfoPub = _nh.advertise<sensor_msgs::CameraInfo>(strVar, 1);
}

UvcCamNode::~UvcCamNode()
{
  _nh.shutdown();
  delete _cam;
  delete _rate;
  delete _imgBuf;
}

void UvcCamNode::start(void)
{
  _cam->connect();
  if(_compressed)
    _cam->setFormat(_width, _height, V4L2_PIX_FMT_MJPEG);
  else
    _cam->setFormat(_width, _height, V4L2_PIX_FMT_YUYV);
  _cam->setFramerate(1, _camFrameRate);
  _cam->startStreaming();
  this->run();
}

void UvcCamNode::run(void)
{
  unsigned int seq = 0;
  while(ros::ok())
  {
    unsigned int bytes = 0;
    obvious::EnumCameraError status = _cam->grab(_imgBuf, &bytes);
    if(status != obvious::CAMSUCCESS)
    {
      std::cout << __PRETTY_FUNCTION__ << " error grabbing camera!\n";
      exit(1);
    }
    if(_compressed)
    {
      sensor_msgs::CompressedImage image;
      image.header.stamp = ros::Time::now();
      image.header.seq   = seq;
      image.header.frame_id = _camId;
      image.data.resize(bytes + EST_HEADER);
      memcpy(image.data.data(), _imgBuf, bytes + EST_HEADER);
      _imagePub.publish(image);
    }
    else
    {
      sensor_msgs::Image image;
      image.header.stamp = ros::Time::now();
      image.header.seq   = seq;
      image.height       = _height;
      image.width        = _width;
      image.step         = 3 * _width;
      image.encoding = sensor_msgs::image_encodings::RGB8;
      image.data.resize(_height * _width * 3);
      memcpy(image.data.data(), _imgBuf,_height * _width * 3);
      _imagePub.publish(image);
    }
    sensor_msgs::CameraInfo camInfo;
    camInfo.header.seq = seq++;
    camInfo.header.stamp = ros::Time::now();
    camInfo.height = _height;
    camInfo.width  = _width;
    _imageInfoPub.publish(camInfo);
    _rate->sleep();
  }
}
