#include <stdlib.h>
#include <iostream>

#include "StateMachine.h"
#include "StatePing.h"
#include "StatePong.h"

namespace robotws
{

StatePing::StatePing(StateMachine* machine) : StateBase(machine)
{

}

StatePing::~StatePing()
{

}

void StatePing::onEntry()
{
  std::cout << "Enter Ping state";
}

void StatePing::onActive()
{
  std::cout << " -> Ping" << std::flush;

  if(rand()%100<30)
    _machine->transitionToState(new StatePong(_machine));
}

void StatePing::onExit()
{
  std::cout << " ... leaving" << std::endl << std::flush;
}

} /* end namespace */
