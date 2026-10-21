#include "fer_ctrl/commands/command_stop.hpp"

namespace fer {
namespace commands {

fsm::retval Stop::Execute()
{
    robotInfo->velocityGoal.setZero();
    const std::string currentState = fsm_->GetCurrentStateName();
    if (currentState == ID::states::disabled || currentState == ID::states::fault
        || currentState == ID::states::parked) {
        return fsm::ok;
    }
    robotInfo->status = "Stop accepted";
    if (currentState == ID::states::enabling) {
        return fsm_->SetNextState(ID::states::disabled);
    }
    return fsm_->SetNextState(ID::states::idle);
}

}
}
