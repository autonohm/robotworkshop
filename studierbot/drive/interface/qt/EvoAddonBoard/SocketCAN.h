#ifndef _SOCKETCAN_H_
#define _SOCKETCAN_H_

#include <linux/can.h>
#include <linux/can/raw.h>
#include <string>
#include <vector>
#include <thread>
#include <mutex>

#include "SocketCANObserver.h"

/**
 * @class SocketCAN
 * @brief CAN communication class. This class uses a threaded listener and observer pattern notifying observer class instances.
 * @author Stefan May
 * @date 13.05.2018
 */
class SocketCAN
{
public:
  /**
   * Constructor
   * @param[in] devFile device file link to CAN interface
   */
  SocketCAN(std::string devFile);

  /**
   * Destructor
   */
  ~SocketCAN();

  /**
   * The SocketCAN class instance reads all CAN data packets and distributes them according to the observer IDs.
   * @param[in] observer Observer instance, which should be notified when data is available.
   * @return success==true
   */
  bool registerObserver(SocketCANObserver* observer);

  /**
   * Open CAN interface.
   * @param[in] port CAN interface name specified with slcand.
   * @return success==true
   */
  bool openPort(const char* port);

  /**
   * Send CAN frame.
   * @param[in] frame CAN frame
   */
  bool send(struct can_frame* frame);

  /**
   * Start listener thread.
   * @return success==true, failure==false (e.g. when listener is already running)
   */
  bool startListener();

  /**
   * Terminate listener thread
   */
  void stopListener();

  /**
   * Close device file link.
   * @return success==true
   */
  bool closePort();

private:

  bool listener();

  int _soc;

  bool _listenerIsRunning;

  bool _shutDownListener;

  std::vector<SocketCANObserver*> _observers;

  std::thread* _thread;

  std::mutex _mutex;
};

#endif //_SOCKETCAN_H_
