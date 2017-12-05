#include <stdlib.h>
#include <iostream>

#include "StateMachine.h"
#include "StatePong.h"
#include "StatePing.h"

namespace robotws
{

StatePong::StatePong(StateMachine* machine) : StateBase(machine)
{

}

StatePong::~StatePong()
{

}

void StatePong::onEntry()
{
  std::cout << "Enter Pong state" << std::flush;
}

void StatePong::onActive()
{
  std::cout << " -> Pong" << std::flush;

  if(rand()%100<30)
    _machine->transitionToState(new StatePing(_machine));
}

void StatePong::onExit()
{
  std::cout << " ... leaving" << std::endl << std::flush;
}

} /* end namespace */
