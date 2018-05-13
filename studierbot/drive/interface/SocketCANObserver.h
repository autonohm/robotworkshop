#ifndef _SOCKETCANOBSERVER_H_
#define _SOCKETCANOBSERVER_H_

#include <linux/can.h>

class SocketCANObserver
{
public:
  SocketCANObserver();

  virtual ~SocketCANObserver();

  void setCANId(canid_t id);

  canid_t getCANId();

  virtual void notify(struct can_frame* frame) = 0;

private:

  canid_t _canid;
};

#endif // _SOCKETCANOBSERVER_H_
