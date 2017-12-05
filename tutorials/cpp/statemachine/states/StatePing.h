#ifndef STATEPING_H_
#define STATEPING_H_

#include "StateBase.h"

namespace robotws
{

/**
 * @class StatePing
 * @brief Example state, transition to Pong state
 * @author Stefan May
 */
class StatePing : public StateBase
{
public:

  /**
   * Constructor
   */
  StatePing(StateMachine* machine);

  /**
   * Destructor
   */
  virtual ~StatePing();

  /**
   * Called once when activated
   */
  void onEntry();

  /**
   * Process method (step-wise, never block this method)
   */
  void onActive();

  /**
   * Called once when left
   */
  void onExit();

private:

};

} /* end namespace */

#endif /* STATEPING_H_ */
