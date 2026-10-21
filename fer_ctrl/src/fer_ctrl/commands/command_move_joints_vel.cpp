#include "fer_ctrl/commands/command_move_joints_vel.hpp"
#include "fer_ctrl/states/state_move_joints_vel.hpp"

namespace fer {
namespace commands {

fsm::retval MoveJointsVel::Execute()
{
    auto state = std::dynamic_pointer_cast<states::MoveJointsVel>(state_);
    if (!state->SetJointsGoal(request_)) {
        return fsm::fail;
    }
    robotInfo->status = "Joint velocity command accepted";
    return fsm_->SetNextState(ID::states::move_joints_vel);
}

}
}
