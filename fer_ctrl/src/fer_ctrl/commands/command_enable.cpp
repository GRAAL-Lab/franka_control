#include "fer_ctrl/commands/command_enable.hpp"

namespace fer {
namespace commands {

fsm::retval Enable::Execute()
{
    if (!robotInfo->feedback.ready || robotInfo->feedback.hasError) {
        robotInfo->status = "Robot is not ready; resolve the error and recover first";
        return fsm::fail;
    }
    if (fsm_->GetCurrentStateName() == ID::states::idle && robotInfo->feedback.controlActive) {
        robotInfo->status = "Controller already enabled";
        return fsm::ok;
    }
    robotInfo->status = "Waiting for driver control loop";
    return fsm_->SetNextState(ID::states::enabling);
}

}
}
