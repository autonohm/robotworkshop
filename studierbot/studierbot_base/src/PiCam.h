#ifndef PICAM_H_
#define PICAM_H_


#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "raspicam/raspicam.h"

class PiCam
{
public:

  PiCam();

  ~PiCam();

  /**
   * @fn void start(const unsigned int frames = 10)
   * @brief 
   * @param[in] const unsigned int rate  ->  rate of the working loop in [1/s]
   */
  void start(const unsigned int rate = 10);

private:

	/**
	 * @fn void run()
	 * @brief main loop (blocking method)
	 * @param[in, out] void
	 */
	void run();


  ros::Rate* _rate;
  ros::NodeHandle _nh;
  image_transport::Publisher _pub;
  raspicam::RaspiCam* _cam;
  unsigned char* _img;
};

#endif
