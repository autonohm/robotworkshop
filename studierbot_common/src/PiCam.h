#ifndef PICAM_H_
#define PICAM_H_

#include "raspicam/raspicam.h"

class PiCam
{
public:

  PiCam();

  virtual ~PiCam();

  /**
   * @fn void run(const unsigned int frames = 10)
   * @brief main loop (blocking method)
   * @param[in] const unsigned int rate  ->  rate of the working loop in [1/s]
   */
  void run(const unsigned int rate = 10);

};

#endif
