#include "fer_ctrl/commands/command_move_joints_pos.hpp"
#include "fer_ctrl/states/state_move_joints_pos.hpp"

namespace fer {
namespace commands {

fsm::retval MoveJointsPos::Execute()
{
    auto state = std::dynamic_pointer_cast<states::MoveJointsPos>(state_);
    if (!state->SetJointsGoal(request_)) {
        return fsm::fail;
    }
    robotInfo->status = "Joint position command accepted";
    return fsm_->SetNextState(ID::states::move_joints_pos);
}

}
}
