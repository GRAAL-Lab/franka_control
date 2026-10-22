/** @file state_parking.hpp */

#ifndef FER_CTRL_STATE_PARKING_HPP
#define FER_CTRL_STATE_PARKING_HPP

#include "fer_ctrl/states/state_move_joints_pos.hpp"

namespace fer {
namespace states {

/** @brief Joint-position motion to the configured parked posture. */
class Parking : public MoveJointsPos {
public:
    /** @brief Create the state with its default action and completion behavior. */
    Parking();
    /**
     * @brief Select the configured parked joint goal before entering the motion state.
     * @return Result of GenericState::OnEntry().
     */
    fsm::retval OnEntry() override;
};

}
}

#endif
