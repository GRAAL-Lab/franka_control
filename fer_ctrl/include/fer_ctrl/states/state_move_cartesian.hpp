#ifndef FER_CTRL_STATE_MOVE_CARTESIAN_HPP
#define FER_CTRL_STATE_MOVE_CARTESIAN_HPP

#include "fer_ctrl/states/generic_state.hpp"

namespace fer {
namespace states {

class MoveCartesian : public GenericState {
public:
    MoveCartesian();
    bool SetCartesianGoal(const CommandRequest& request);
    fsm::retval Execute() override;
};

}
}

#endif
