#ifndef FER_CTRL_STATE_MOVE_JOINTS_VEL_HPP
#define FER_CTRL_STATE_MOVE_JOINTS_VEL_HPP

#include "fer_ctrl/states/generic_state.hpp"

namespace fer {
namespace states {

class MoveJointsVel : public GenericState {
public:
    MoveJointsVel();
    bool SetJointsGoal(const CommandRequest& request);
    fsm::retval Execute() override;
};

}
}

#endif
