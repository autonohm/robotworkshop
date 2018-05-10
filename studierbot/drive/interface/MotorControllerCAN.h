#ifndef _MOTORCONTROLLERCAN_H_
#define _MOTORCONTROLLERCAN_H_

#include "MotorController.h"
#include <linux/can.h>
#include <linux/can/raw.h>

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
   * Get standard parameters for CAN motor interface
   * @return Default MotorParams
   */
  static MotorParams getStandardParameters();

  /**
   * Enable device
   * @return enable state
   */
  virtual bool enable();

  /**
   * Set motor revolutions per minute
   * @param[in] rpm set point value, this device supports up to 16 channels
   */
  virtual void setRPM(std::map<MotorControllerChannel, float> rpm);

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

  bool openPort(const char *port);

  bool sendPort(struct can_frame *frame);

  void readPort();

  int closePort();

  int _soc;

  can_frame _cf;
};

#endif /* _MOTORCONTROLLERCAN_H_ */
