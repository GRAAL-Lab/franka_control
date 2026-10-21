#ifndef FER_CTRL_STATE_UNPARKING_HPP
#define FER_CTRL_STATE_UNPARKING_HPP

#include "fer_ctrl/states/state_move_joints_pos.hpp"

namespace fer {
namespace states {

class Unparking : public MoveJointsPos {
public:
    Unparking();
    fsm::retval OnEntry() override;
};

}
}

#endif
