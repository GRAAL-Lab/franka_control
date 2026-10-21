#ifndef FER_CTRL_STATE_MOVE_JOINTS_POS_HPP
#define FER_CTRL_STATE_MOVE_JOINTS_POS_HPP

#include "fer_ctrl/states/generic_state.hpp"

namespace fer {
namespace states {

class MoveJointsPos : public GenericState {
public:
    MoveJointsPos();
    bool SetJointsGoal(const CommandRequest& request);
    fsm::retval Execute() override;

protected:
    MoveJointsPos(
        const std::string& stateName, const std::string& actionName, const std::string& completionState);
    std::string completionState_;
};

}
}

#endif
