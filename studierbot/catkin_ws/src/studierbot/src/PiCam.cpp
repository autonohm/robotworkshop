#include <ros/ros.h>
#include "PiCam.h"



PiCam::PiCam() :
  _rate(0),
  _cam(new raspicam::RaspiCam),
  _img(NULL)
{
  _cam->setWidth(320);
  _cam->setHeight(240);
  _cam->setFormat(raspicam::RASPICAM_FORMAT_GRAY);
  _cam->setRotation(90);

  // Standard publisher (uncompressed)
  //ros::Publisher pub = n.advertise<sensor_msgs::Image>("pi_cam_image",1);

  // image_transport publisher (uncompressed or compressed)
  image_transport::ImageTransport it(_nh);
  _pub = it.advertise("pi_cam_image", 1);

  std::string key;
  if(ros::param::search("pi_cam_image/compressed/format", key))
  {
    ros::param::set(key, "png");
  }
  if(ros::param::search("pi_cam_image/compressed/png_level", key))
  {
    ros::param::set(key, 9);
  }
}

PiCam::~PiCam()
{
  delete _rate;
}

void PiCam::start(const unsigned int loopRate)
{
  delete _rate;
  _rate = new ros::Rate(loopRate);
  //-----------------------------


  //----- Initialize camera -----
  //raspicam::RaspiCam cam;

  if ( !_cam->open())
  {
    std::cerr<<"Error opening camera"<<std::endl;
    return;
  }

  // wait a while until camera stabilizes
  std::cout<<"Sleeping for 3 secs"<<std::endl;
  sleep(3);

  _cam->grab();
  //-----------------------------


  // allocate memory
  _img = new unsigned char[_cam->getImageTypeSize ( raspicam::RASPICAM_FORMAT_GRAY )];

  _cam->retrieve(_img, raspicam::RASPICAM_FORMAT_IGNORE);
	std::cout<<"Hoehe: "<<_cam->getHeight()<<std::endl<<"Breite: "<<_cam->getWidth()<<std::endl;
	this->run();
}

void PiCam::run()
{
	unsigned int cnt = 0;

  sensor_msgs::Image* msg = new sensor_msgs::Image;
  msg->is_bigendian       = 0;
  msg->encoding           = sensor_msgs::image_encodings::MONO8;

  msg->width              = _cam->getWidth();
  msg->height             = _cam->getHeight();
  msg->step               = _cam->getWidth();// * 3;
  msg->data.resize(msg->step * msg->height);

  unsigned int image_size = _cam->getWidth() * _cam->getHeight();
  while(ros::ok())
  {
		//get raspi image
  	if(_cam->grab())
    {
      _cam->retrieve(_img,raspicam::RASPICAM_FORMAT_IGNORE);

      msg->header.stamp    = ros::Time::now();
      msg->header.frame_id = cnt++;

      //memcpy(&msg->data[0], img, cam.getWidth() * cam.getHeight() * 3);

      for(unsigned int i=0; i<image_size; i++)
      {
        unsigned int idx = i; 
				msg->data[i]   = _img[idx];

      }

      _pub.publish(*msg);
    }

    _rate->sleep();
  }

  delete msg;

}
