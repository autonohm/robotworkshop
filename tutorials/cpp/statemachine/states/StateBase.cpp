#include "StateBase.h"
#include <stddef.h>

namespace robotws
{

StateBase::StateBase(StateMachine* machine)
{
  _machine = machine;
}

StateBase::~StateBase()
{

}

} /* end namespace */

