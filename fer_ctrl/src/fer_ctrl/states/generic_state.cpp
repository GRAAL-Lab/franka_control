#include "fer_ctrl/states/generic_state.hpp"

#include <stdexcept>

namespace fer {
namespace states {

GenericState::GenericState(const std::string& stateName, const std::string& actionName)
    : stateName_(stateName)
    , actionName_(actionName)
{
}

void GenericState::ConfigureStateFromFile(libconfig::Config& configuration)
{
    const auto& settings = configuration.getRoot()["states"];
    if (settings.exists(stateName_)) {
        settings[stateName_.c_str()].lookupValue("action", actionName_);
    }
    actionManager->GetAction(actionName_);
}

fsm::retval GenericState::OnEntry()
{
    robotInfo->stateEntryTime = robotInfo->currentTime;
    bool moving = stateName_ == ID::states::move_joints_pos || stateName_ == ID::states::move_joints_vel
        || stateName_ == ID::states::move_cartesian || stateName_ == ID::states::parking
        || stateName_ == ID::states::unparking;
    if (!actionManager->SetAction(actionName_, moving && configuration->actionTransitionDuration > 0.0)) {
        throw std::runtime_error("Cannot select task-priority action: " + actionName_);
    }
    return fsm::ok;
}

fsm::retval GenericState::ChangeState(const std::string& nextState, const std::string& message)
{
    robotInfo->status = message;
    return fsm_->SetNextState(nextState);
}

}
}
