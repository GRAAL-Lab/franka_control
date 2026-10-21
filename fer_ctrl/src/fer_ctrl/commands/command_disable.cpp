#include "fer_ctrl/commands/command_disable.hpp"

namespace fer {
namespace commands {

fsm::retval Disable::Execute()
{
    robotInfo->velocityGoal.setZero();
    robotInfo->status
        = request_.commandType == "recover" ? "Recovery requested; enable after completion" : "Disabled";
    return fsm_->SetNextState(ID::states::disabled);
}

}
}
