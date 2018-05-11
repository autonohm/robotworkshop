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
   * Set pulse width modulated signal
   * @param[in] rpm pulse width in range [-100;100], this device supports up to 32 channels
   * @param[out] revolutions per minute (RPM)
   * @return success
   */
  virtual bool setPWM(std::vector<int> pwm, std::vector<float> &rpm);

  /**
   * Set motor revolutions per minute
   * @param[in] rpmIn set point value, this device supports up to 32 channels
   * @param[out] rpmOut rotational speed in revolutions per minute (RPM)
   * @return success
   */
  virtual bool setRPM(std::vector<float> rpmIn, std::vector<float> &rpmOut);

  /**
   * Stop motors
   */
  void stop();

private:

  bool openPort(const char *port);

  bool sendPort(struct can_frame *frame);

  bool readPort(float* rpm1, float* rpm2);

  int closePort();

  int _soc;

  can_frame _cf;
};

#endif /* _MOTORCONTROLLERCAN_H_ */
