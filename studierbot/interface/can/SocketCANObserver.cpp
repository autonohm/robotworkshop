#include "SocketCANObserver.h"

SocketCANObserver::SocketCANObserver()
{
  _canid = 0x0;
}

SocketCANObserver::~SocketCANObserver()
{

}

void SocketCANObserver::setCANId(canid_t canid)
{
  _canid = canid;
}

canid_t SocketCANObserver::getCANId()
{
  return _canid;
}
