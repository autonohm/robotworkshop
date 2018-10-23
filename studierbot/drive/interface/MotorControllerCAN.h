#ifndef _MOTORCONTROLLERCAN_H_
#define _MOTORCONTROLLERCAN_H_

#include "SocketCAN.h"
#include <vector>

enum CanResponse
{
  CAN_RESPONSE_RPM,
  CAN_RESPONSE_POS
};

struct MotorParams
{
  unsigned short   frequencyScale;
  float            inputWeight;
  unsigned char    maxPulseWidth;
  unsigned short   timeout;
  float            gearRatio;
  float            encoderRatio;
  float            rpmMax;
  enum CanResponse responseMode;
  float            kp;
  float            ki;
  float            kd;
  int              antiWindup;

  /**
   * Standard constructor assigns default parameters
   */
  MotorParams()
  {
    frequencyScale = 32;
    inputWeight    = 1.f;
    maxPulseWidth  = 63;
    timeout        = 100;
    gearRatio      = 0.f;
    encoderRatio   = 0.f;
    rpmMax         = 0.f;
    responseMode   = CAN_RESPONSE_RPM;
    kp             = 1.f;
    ki             = 0.f;
    kd             = 0.f;
    antiWindup     = 1;
  }

  /**
   * Copy constructor
   * @param[in] p parameter instance to be copied
   */
  MotorParams(const MotorParams &p)
  {
    frequencyScale = p.frequencyScale;
    inputWeight    = p.inputWeight;
    maxPulseWidth  = p.maxPulseWidth;
    timeout        = p.timeout;
    gearRatio      = p.gearRatio;
    encoderRatio   = p.encoderRatio;
    rpmMax         = p.rpmMax;
    responseMode   = p.responseMode;
    kp             = p.kp;
    ki             = p.ki;
    kd             = p.kd;
    antiWindup     = p.antiWindup;
  }
};

/**
 * @class MotorControllerCAN
 * @brief CAN interface for Evocortex Centipede motor controller.
 * @author Stefan May
 * @date 13.05.2018
 */
class MotorControllerCAN : public SocketCANObserver
{
public:
  /**
   * Constructor
   * @param[in] can SocketCAN instance
   * @param[in] params motor parameters
   */
  MotorControllerCAN(SocketCAN* can, unsigned int canID, MotorParams params);

  /**
   * Destructor
   */
  ~MotorControllerCAN();

  /**
   * Enable device
   * @return successful transmission of enable command
   */
  bool enable();

  /**
   * Disable device
   * @return successful transmission of disabling command
   */
  bool disable();

  /**
   * Send synchronization signal, resetting a counter in order to have a common time basis on all controllers
   * @return successful transmission of synchronization signal
   */
  bool broadcastExternalSync();

  /**
   * Configure response of motor controller (revolutions per minute or position)
   * @param[in] mode response mode
   * @return successful transmission of configure command
   */
  bool configureResponse(enum CanResponse mode);

  /**
   * Get assigned canID via constructor
   * @return ID
   */
  unsigned short getCanId();

  /**
   * Set timeout interval. The motor controller needs frequently transmitted commands.
   * If the time span between two commands is longer than this timeout interval, the device is disabled.
   * The user needs to send an enabling command again.
   * @param[in] timeoutInMillis timeout interval in milliseconds
   * @return true==successful CAN transmission
   */
  bool setTimeout(unsigned short timeoutInMillis);

  /**
   * Accessor to timeout parameter. See commets of mutator for more information.
   * @return timeout in milliseconds
   */
  unsigned short getTimeout();

  /**
   * Set gear ratio (factor between motor and wheel revolutions)
   * @param[in] gearRatio (motor rev) / (wheel rev) for motor 1 and 2
   * @return true==successful CAN transmission
   */
  bool setGearRatio(float gearRatio);

  /**
   * Accessor to gear ratio parameter
   * @return gearRatio (motor rev) / (wheel rev) for motor 1 and 2
   */
  float getGearRatio();

  /**
   * Set number of encoder ticks per motor revolution
   * @param[in] encoderTicksPerRev encoder ticks per motor revolution for motor 1 and 2
   * @return true==successful CAN transmission
   */
  bool setEncoderTicksPerRev(float encoderTicksPerRev);

  /**
   * Accessor to parameter representing encoder ticks per motor revolution
   * @return encoder ticks per motor revolution for motor 1 and 2
   */
  float getEncoderTicksPerRev();

  /**
   * Set scaling parameter for PWM frequency. The base frequency is 500kHz, of which one can apply a fractional amount, e.g.
   * 10 => 50kHz
   * 20 => 25kHz
   * Default is 32 => 15,625kHz
   * Important: This value can only be changed before the motor controllers gets enabled.
   * @param[in] scale denominator d of term 1/d x 500kHz
   */
  bool setFrequencyScale(unsigned short scale);

  /**
   * Accessor to frequency scaling parameter, see mutator for details.
   * @return scale denominator d of term 1/d x 500kHz
   */
  unsigned short getFrequencyScale();

  /**
   * The PWM signal can be adjusted in the range from [-127;127] which is equal to [-100%;100%].
   * To limit the possible output, one can set a different value between [0;127],
   * which is symmetrically applied to the positive and negative area, e.g. 32 => [-25%;25%]
   * The default value is: 63 => [-50%;50%]
   * @param[in] pulse pulse width limit in range of [-127;127]
   */
  bool setMaxPulseWidth(unsigned char pulse);

  /**
   * Accessor to pulse width limit, see mutator for details.
   * @return pulse width limit
   */
  unsigned char getMaxPulseWidth();

  /**
   * Set pulse width modulated signal
   * @param[in] rpm pulse width in range [-100;100], this device supports 2 channels
   * @return success
   */
  bool setPWM(int pwm[2]);

  /**
   * Set motor revolutions per minute
   * @param[in] rpm set point value, this device supports 2 channels
   * @return success
   */
  bool setRPM(float rpm[2]);

  /**
   * Get either motor revolutions per minute or motor position (encoder ticks). This depends on the configuration canResponseMode.
   * @param[out] response revolutions per minute for motor 1 and 2 / position of motor 1 and 2. This is a modulo 2^15 value.
   */
  void getWheelResponse(float response[2]);

  /**
   * Set proportional factor of PID controller
   * @param[in] kp proportional factor
   * @return success
   */
  bool setKp(float kp);

  /**
   * Accessor to proportional factor of PID controller
   * @return proportional factor
   */
  float getKp();

  /**
   * Set integration factor of PID controller
   * @param[in] ki integration factor
   * @return success
   */
  bool setKi(float ki);

  /**
   * Accessor to integration factor of PID controller
   * @return integration factor
   */
  float getKi();

  /**
   * Set differential factor of PID controller
   * @param[in] kd differential factor
   * @return success
   */
  bool setKd(float kd);

  /**
   * Accessor to differential factor of PID controller
   * @return differential factor
   */
  float getKd();

  /**
   * Set weight of input filter. Input values f are filtered with f'=weight*f'+(1-weight)*f.
   * @param[in] filtering weight. A value of 0 disables the filter. The value must be in the range of [0;1[
   */
  bool setInputWeight(float weight);

  /**
   * Accessor to weight of input filter. See comments of mutator for more information.
   * @return weight of input filter
   */
  float getInputWeight();

  /**
   * Wait for synchronization after a new PWM or RPM value is set.
   * @param[in] timeoutInMillis timeout period in milliseconds. A value of 0 disables the timeout check.
   * @return true==successful synchronization
   */
  bool waitForSync(unsigned int timeoutInMillis=100);

  /**
   * Stop motors
   */
  void stop();

protected:

private:

  bool sendFloat(int cmd, float f);

  /**
   * Implementation of inherited method from SocketCANObserver. This class is getting notified by the SocketCAN,
   * as soon as messages of interest arrive (having the desired CAN ID).
   * @param[in] frame CAN frame
   */
  void notify(struct can_frame* frame);

  SocketCAN*       _can;

  can_frame        _cf;

  unsigned long    _idSyncSend;

  unsigned long    _idSyncReceive;

  MotorParams      _params;

  enum CanResponse _responseMode;

  float            _rpm[2];

  short            _pos[2];
};

#endif /* _MOTORCONTROLLERCAN_H_ */
