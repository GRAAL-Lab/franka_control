#include "fer_ctrl/commands/command_disable.hpp"

namespace fer {
namespace commands {

fsm::retval Disable::Execute()
{
    robotInfo->velocityGoal.setZero();
    if (request_.commandType == "recover") {
        robotInfo->status = "Recovery requested; enable after completion";
        return fsm_->SetNextState(ID::states::disabled);
    }
    if (fsm_->GetCurrentStateName() == ID::states::disabling) {
        return fsm::ok;
    }
    robotInfo->status = "Waiting for driver control loop to stop";
    return fsm_->SetNextState(ID::states::disabling);
}

}
}
