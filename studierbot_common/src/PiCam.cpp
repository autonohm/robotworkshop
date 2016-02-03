#include "PiCam.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>

#include <image_transport/image_transport.h>

PiCam::PiCam()
{

}

PiCam::~PiCam()
{

}

void PiCam::run(const unsigned int loopRate)
{
  //----- ROS transportation layer -----
  ros::NodeHandle n;

  // Standard publisher (uncompressed)
  //ros::Publisher pub = n.advertise<sensor_msgs::Image>("pi_cam_image",1);

  // image_transport publisher (uncompressed or compressed)
  image_transport::ImageTransport it(n);
  image_transport::Publisher pub = it.advertise("pi_cam_image", 1);

  ros::Rate rate(loopRate);
  //-----------------------------


  //----- Initialize camera -----
  raspicam::RaspiCam cam;

  if ( !cam.open())
  {
    std::cerr<<"Error opening camera"<<std::endl;
    return;
  }

  // wait a while until camera stabilizes
  std::cout<<"Sleeping for 3 secs"<<std::endl;
  sleep(3);

  cam.grab();
  //-----------------------------


  // allocate memory
  unsigned char* img = new unsigned char[cam.getImageTypeSize ( raspicam::RASPICAM_FORMAT_RGB )];

  cam.retrieve(img, raspicam::RASPICAM_FORMAT_IGNORE);

  sensor_msgs::Image* msg = new sensor_msgs::Image;
  msg->is_bigendian       = 0;
  msg->encoding           = sensor_msgs::image_encodings::RGB8;

  msg->width              = cam.getWidth();
  msg->height             = cam.getHeight();
  msg->step               = cam.getWidth() * 3;
  msg->data.resize(msg->step * msg->height);


  unsigned int cnt = 0;
  while(ros::ok())
  {
    cam.grab();
    cam.retrieve(img,raspicam::RASPICAM_FORMAT_IGNORE);

    msg->header.stamp    = ros::Time::now();
    msg->header.frame_id = cnt++;

    memcpy(&msg->data[0], img, cam.getWidth() * cam.getHeight() * 3);

    pub.publish(*msg);

    rate.sleep();
  }

  delete [] img;
  delete msg;

}
