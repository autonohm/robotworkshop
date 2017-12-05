#ifndef STATEPONG_H_
#define STATEPONG_H_

#include "StateBase.h"

namespace robotws
{

/**
 * @class StatePong
 * @brief Example state, transition to Ping state
 * @author Stefan May
 */
class StatePong: public StateBase
{
public:

  /**
   * Constructor
   */
  StatePong(StateMachine* machine);

  /**
   * Destructor
   */
  virtual ~StatePong();

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

#endif /* STATEPONG_H_ */
