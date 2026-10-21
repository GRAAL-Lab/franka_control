#ifndef FER_CTRL_STATE_PARKING_HPP
#define FER_CTRL_STATE_PARKING_HPP

#include "fer_ctrl/states/state_move_joints_pos.hpp"

namespace fer {
namespace states {

class Parking : public MoveJointsPos {
public:
    Parking();
    fsm::retval OnEntry() override;
};

}
}

#endif
