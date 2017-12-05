#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_

#include "StateBase.h"

/**
 * @namespace  robotws
 */
namespace robotws
{

/**
 * @class   StateMachine
 * @brief   State machine main class
 * @author  Stefan May
 * @date    5.12.2017
 */
class StateMachine
{

public:

  /**
   * Constructor
   */
  StateMachine();

  /**
   * Destructor
   */
  virtual ~StateMachine();

  /**
   * Awake state machine to call active states
   */
  void awake();

  /**
   * Activate new state instance. Instance is deleted with next transition.
   * @param nextState next state to activate
   */
  void transitionToState(StateBase* nextState);


private:

  StateBase* _currentState;

  StateBase* _nextState;

};

} // end namespace

#endif
