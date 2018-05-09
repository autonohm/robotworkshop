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
   * Set motor revolutions per minute
   * @param[in] rpm set point value, this device supports up to 6 channels
   */
  void setRPM(std::vector<float> rpm);

  /**
   * Set motor revolutions per minute
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

  /**
   * Send commands to motor shield
   * @param[in] cmd command byte
   * @param[in] param parameter
   * @param[in] echo verbosity of function, true provides command line output
   */
  template<typename T>
  bool sendToMotorshield(char cmd, T param, bool echo);
  bool sendToMotorshieldS(char cmd, short param[2], bool echo);
  bool sendToMotorshieldI(char cmd, int param, bool echo);
  bool sendToMotorshieldF(char cmd, float param, bool echo);

  speed_t     _baud;
  char        _bufCmd[14];
  char        _bufResponse[13];
  SerialPort* _com;
  short       _rpm[6];
};

#endif /* _MOTORCONTROLLERSERIAL_H_ */
