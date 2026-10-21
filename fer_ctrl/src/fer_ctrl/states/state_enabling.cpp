#include "fer_ctrl/states/state_enabling.hpp"

namespace fer {
namespace states {

Enabling::Enabling()
    : GenericState(ID::states::enabling)
{
}

fsm::retval Enabling::Execute()
{
    if (robotInfo->feedback.controlActive) {
        return ChangeState(ID::states::idle, "Enabled");
    }
    if (robotInfo->currentTime - robotInfo->stateEntryTime > conf->enableTimeout) {
        return ChangeState(ID::states::fault, "Driver enable timed out");
    }
    return fsm::ok;
}

}
}
