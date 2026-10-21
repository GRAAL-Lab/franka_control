#include "fer_ctrl/states/state_parking.hpp"

namespace fer {
namespace states {

Parking::Parking()
    : MoveJointsPos(ID::states::parking, "ParkUnpark", ID::states::parked)
{
}

fsm::retval Parking::OnEntry()
{
    robotInfo->jointsGoal = conf->parkedJointsPos;
    return GenericState::OnEntry();
}

}
}
