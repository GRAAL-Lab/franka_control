#include "fer_ctrl/commands/command_enable.hpp"

namespace fer {
namespace commands {

fsm::retval Enable::Execute()
{
    if (!robotInfo->feedback.ready || robotInfo->feedback.hasError) {
        robotInfo->status = "Robot is not ready; resolve the error and recover first";
        return fsm::fail;
    }
    robotInfo->status = "Waiting for driver control loop";
    return fsm_->SetNextState(ID::states::enabling);
}

}
}
