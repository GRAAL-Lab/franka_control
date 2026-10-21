#include "fer_ctrl/states/state_idle.hpp"

namespace fer {
namespace states {

Idle::Idle()
    : GenericState(ID::states::idle)
{
}

fsm::retval Idle::Execute()
{
    robotInfo->velocityGoal.setZero();
    return fsm::ok;
}

}
}
