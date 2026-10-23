#include "fer_ctrl/states/state_disabling.hpp"

namespace fer {
namespace states {

Disabling::Disabling()
    : GenericState(ID::states::disabling)
{
}

fsm::retval Disabling::Execute()
{
    const double feedbackAge = robotInfo->currentTime - robotInfo->lastFeedbackTime;
    const bool feedbackFresh = robotInfo->receivingFeedback && robotInfo->feedback.connected
        && robotInfo->lastFeedbackTime >= robotInfo->stateEntryTime
        && feedbackAge >= 0.0 && feedbackAge <= configuration->feedbackTimeout;
    if (feedbackFresh && !robotInfo->feedback.controlActive) {
        return ChangeState(ID::states::disabled, "Disabled");
    }
    if (robotInfo->currentTime - robotInfo->stateEntryTime > configuration->disableTimeout) {
        return ChangeState(ID::states::fault, "Driver disable timed out");
    }
    return fsm::ok;
}

}
}
