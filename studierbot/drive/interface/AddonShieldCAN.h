#ifndef _ADDONSHIELDCAN_H_
#define _ADDONSHIELDCAN_H_

#include "SocketCAN.h"
#include <vector>

/**
 * @class AddonShieldCAN
 * @brief CAN interface for Evocortex Centipede add-on shield.
 * @author Stefan May
 * @date 01.12.2018
 */
class AddonShieldCAN : public SocketCANObserver
{
public:
  /**
   * Constructor
   * @param[in] can SocketCAN instance
   * @param[in] verbosity verbosity output flag
   */
  AddonShieldCAN(SocketCAN* can, bool verbosity=0);

  /**
   * Destructor
   */
  ~AddonShieldCAN();

  /**
   * Enable device
   * @param[in] channel channel in range [2..4], channel 1 is always enabled
   * @return successful transmission of enabling command
   */
  bool enable(unsigned char channel);

  /**
   * Disable device
   * @param[in] channel channel in range [2..4], channel 1 cannot be disabled
   * @return successful transmission of disabling command
   */
  bool disable(unsigned char channel);

  /**
   * Get supply voltage
   * @return voltage
   */
  float getVoltage();

  /**
   * Set automatic switch-off threshold
   * @param[in] channel channel
   * @param[in] threshold threshold in V
   */
  bool setThreshold(unsigned char channel, float threshold);

  /**
   * Set PWM frequency
   * @return successful transmission of command
   */
  bool setPWMFrequency(unsigned short frequency);

  /**
   * Set pulse-width
   * @param[in] channel channel in range [1;4]
   * @param[in] pwm pulse-width modulation in range [0;100]
   * @return successful transmission of command
   */
  bool setPulseWidth(unsigned char channel, unsigned char pwm);

  /**
   * Wait for synchronization after a new PWM or RPM value is set.
   * @param[in] timeoutInMillis timeout period in milliseconds. A value of 0 disables the timeout check.
   * @return true==successful synchronization
   */
  bool waitForSync(unsigned int timeoutInMillis=100);

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

  float _voltage;

};

#endif /* _ADDONSHIELDCAN_H_ */
