#ifndef _SOCKETCAN_H_
#define _SOCKETCAN_H_

#include <linux/can.h>
#include <linux/can/raw.h>
#include <string>
#include <vector>
#include <thread>
#include <mutex>

#include "SocketCANObserver.h"

class SocketCAN
{
public:
  /**
   * Constructor
   * @param[in] devFile device file link to CAN interface
   */
  SocketCAN(std::string devFile);

  ~SocketCAN();

  bool registerObserver(SocketCANObserver* observer);

  bool openPort(const char *port);

  bool send(struct can_frame *frame);

  bool startListener();

  void stopListener();

  int closePort();

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
