#include "fer_ctrl/commands/command_move_cartesian.hpp"
#include "fer_ctrl/states/state_move_cartesian.hpp"

namespace fer {
namespace commands {

fsm::retval MoveCartesian::Execute()
{
    auto state = std::dynamic_pointer_cast<states::MoveCartesian>(state_);
    if (!state->SetCartesianGoal(request_)) {
        return fsm::fail;
    }
    robotInfo->status = "Cartesian command accepted";
    return fsm_->SetNextState(ID::states::move_cartesian);
}

}
}
