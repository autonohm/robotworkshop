#ifndef _SOCKETCANOBSERVER_H_
#define _SOCKETCANOBSERVER_H_

#include <linux/can.h>

/**
 * @class SocketCANObserver
 * @brief Abstract observer class. Derived classes get notified according their CAN bus identifiers.
 * @author Stefan May
 * @date 13.05.2018
 */
class SocketCANObserver
{
public:
  /**
   * Constructor
   */
  SocketCANObserver();

  /**
   * Destructor
   */
  virtual ~SocketCANObserver();

  /**
   * Set CAN bus identifier
   * @param[in] id CAN ID
   */
  void setCANId(canid_t id);

  /**
   * Get CAN bus identifier
   * @return CAN ID
   */
  canid_t getCANId();

  /**
   * Interface declaration for implementation through inherited classes.
   * @params[in] frame CAN frame
   */
  virtual void notify(struct can_frame* frame) = 0;

private:

  canid_t _canid;
};

#endif // _SOCKETCANOBSERVER_H_
