#include "StateMachine.h"
#include <string.h>
#include <iostream>

namespace robotws
{

StateMachine::StateMachine()
{
  _currentState = NULL;
  _nextState = NULL;
}

StateMachine::~StateMachine()
{
  if(_currentState)
  {
    _currentState->onExit();
    delete _currentState;
  }
}

void StateMachine::awake()
{
  // check 1st time awake
  if(!_currentState && _nextState)
  {
    _currentState = _nextState;
    _currentState->onEntry();
    _nextState = NULL;
  }

  if(_currentState)
  {
    _currentState->onActive();

    if(_nextState) // do transition
    {
      // do de-initialization of current state
      _currentState->onExit();
      delete _currentState;
    }
  }

  if(_nextState) // do transition
  {
    // do transition to next state
    _currentState = _nextState;
    _nextState = NULL;
    _currentState->onEntry();
  }
}

void StateMachine::transitionToState(StateBase* nextState)
{
  if(_currentState != nextState)
  {
    if(nextState!=NULL)
    {
      _nextState = nextState;
    }
    else
    {
      std::cout << "Next state instance invalid. Either NULL or already assigned state passed." << std::endl;
    }
  }
}

} // end namespace

