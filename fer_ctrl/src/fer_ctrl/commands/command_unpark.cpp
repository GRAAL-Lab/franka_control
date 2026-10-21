#include "fer_ctrl/commands/command_unpark.hpp"

namespace fer {
namespace commands {

fsm::retval Unpark::Execute()
{
    robotInfo->status = "Moving to unparked posture";
    return fsm_->SetNextState(ID::states::unparking);
}

}
}
