#include "fer_ctrl/commands/command_park.hpp"

namespace fer {
namespace commands {

fsm::retval Park::Execute()
{
    robotInfo->status = "Moving to parked posture";
    return fsm_->SetNextState(ID::states::parking);
}

}
}
