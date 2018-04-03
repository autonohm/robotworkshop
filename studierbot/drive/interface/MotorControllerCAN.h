#ifndef _MOTORCONTROLLERCAN_H_
#define _MOTORCONTROLLERCAN_H_

#include "MotorController.h"

class MotorControllerCAN : public MotorController
{
public:
  /**
   * Constructor
   * @param[in] params gear motor parameters, PID controller parameters
   */
  MotorControllerCAN(MotorParams &params);

  /**
   * Destructor
   */
  ~MotorControllerCAN();

  /**
   * Set revolutions per minute
   * @param[in] rpm 6-channel rpm value
   */
  void setRPM(float rpm[6]);

  /**
   * Get revolutions per minute
   * param[in] idx motor index
   * @return rpm
   */
  float getRPM(unsigned int idx);

  /**
   * Stop motors
   */
  void stop();

private:

  void init();

  int openPort(const char *port);

  int sendPort(struct can_frame *frame);

  void readPort();

  int closePort();

  int _soc;
};

#endif /* _MOTORCONTROLLERCAN_H_ */
