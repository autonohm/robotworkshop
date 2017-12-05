#ifndef STATEBASE_H_
#define STATEBASE_H_

#include <iostream>
#include <map>

/**
 * @namespace  robotws
 */
namespace robotws
{

class StateMachine;

/**
 * @class   StateBase
 * @author  Stefan May
 * @date    5.12.2017
 */
class StateBase
{

public:

  /**
   * Constructor
   */
  StateBase(StateMachine* machine);

  /**
   * Default destructor
   */
  virtual ~StateBase();

  /**
   * Called once when activated
   */
  virtual void onEntry() {};

  /**
   * Called while active
   */
  virtual void onActive() = 0;

  /**
   * Called once when left
   */
  virtual void onExit() {};

protected:

  StateMachine* _machine;

};

} /* end namespace */

#endif /* STATEBASE_H_ */
