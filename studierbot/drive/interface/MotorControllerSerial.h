#ifndef _MOTORCONTROLLERSERIAL_H_
#define _MOTORCONTROLLERSERIAL_H_

#include "SerialPort.h"
#include "MotorController.h"

class MotorControllerSerial : public MotorController
{
public:
  /**
   * Constructor
   * @param[in] params gear motor parameters, PID controller parameters
   */
  MotorControllerSerial(MotorParams &params);

  /**
   * Destructor
   */
  ~MotorControllerSerial();

  /**
   * Get standard parameters for serial motor interface
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
   * @param[in] prm pulse width in range [-100;100]
   * @param[out] revolutions per minute (RPM)
   * @return success
   */
  virtual bool setPWM(std::vector<int> pwm, std::vector<float> &rpm);

  /**
   * Set motor revolutions per minute
   * @param[in] rpmIn set point value, this device supports up to 6 channels
   * @param[out] rpmOut rotational speed in revolutions per minute (RPM)
   * @return success
   */
  virtual bool setRPM(std::vector<float> rpmIn, std::vector<float> &rpmOut);

  /**
   * Stop motors
   */
  void stop();

private:

  /**
   * Send commands to motor shield
   * @param[in] cmd command byte
   * @param[in] param parameter
   * @param[in] echo verbosity of function, true provides command line output
   */
  template<typename T>
  bool sendToMotorshield(char cmd, T param, T* response, bool echo);
  bool sendToMotorshieldS(char cmd, short param[6], short (*response)[6], bool echo);
  bool sendToMotorshieldI(char cmd, int param, int* response, bool echo);
  bool sendToMotorshieldF(char cmd, float param, float* response, bool echo);

  bool        _init;
  speed_t     _baud;
  char        _bufCmd[14];
  char        _bufResponse[13];
  SerialPort* _com;
  short       _rpm[6];
};

#endif /* _MOTORCONTROLLERSERIAL_H_ */
