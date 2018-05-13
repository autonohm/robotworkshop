#ifndef _MOTORCONTROLLERCAN_H_
#define _MOTORCONTROLLERCAN_H_

#include "SocketCAN.h"
#include <vector>

class MotorControllerCAN : public SocketCANObserver
{
public:
  /**
   * Constructor
   * @param[in] can SocketCAN instance
   * @param[in] channel Identifier of CAN node, i.e. channel in range [0;15]
   */
  MotorControllerCAN(SocketCAN* can, unsigned short channel);

  /**
   * Destructor
   */
  ~MotorControllerCAN();

  /**
   * Enable device
   * @return enable state
   */
  bool enable();

  bool setGearRatio(float gearRatio[2]);

  bool setEncoderTicksPerRev(float encoderTicksPerRev[2]);

  /**
   * Set pulse width modulated signal
   * @param[in] rpm pulse width in range [-100;100], this device supports 2 channels
   * @return success
   */
  bool setPWM(int pwm[2]);

  /**
   * Set motor revolutions per minute
   * @param[in] rpmIn set point value, this device supports 2 channels
   * @return success
   */
  bool setRPM(float rpmIn[2]);

  void getRPM(float rpm[2]);

  void notify(struct can_frame* frame);

  bool waitForSync();

  /**
   * Stop motors
   */
  void stop();

private:

  SocketCAN* _can;

  can_frame _cf;

  float _rpm[2];

  unsigned long _idSyncSend;
  unsigned long _idSyncReceive;
};

#endif /* _MOTORCONTROLLERCAN_H_ */
