#include <unistd.h>

#include "StateMachine.h"
#include "StatePing.h"
#include "StatePong.h"

int main(int argc, char* argv[])
{
  robotws::StateMachine machine;
  machine.transitionToState(new robotws::StatePing(&machine));

  while(true)
  {
    machine.awake();
    usleep(500000);
  }

  return 0;
}
