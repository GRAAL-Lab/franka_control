#include "fer_ctrl/states/state_unparking.hpp"

namespace fer {
namespace states {

Unparking::Unparking()
    : MoveJointsPos(ID::states::unparking, "ParkUnpark", ID::states::idle)
{
}

fsm::retval Unparking::OnEntry()
{
    robotInfo->jointsGoal = conf->unparkedJointsPos;
    return GenericState::OnEntry();
}

}
}
