/** @file state_unparking.hpp */

#ifndef FER_CTRL_STATE_UNPARKING_HPP
#define FER_CTRL_STATE_UNPARKING_HPP

#include "fer_ctrl/states/state_move_joints_pos.hpp"

namespace fer {
namespace states {

/** @brief Joint-position motion to the configured unparked posture. */
class Unparking : public MoveJointsPos {
public:
    /** @brief Create the state with its default action and completion behavior. */
    Unparking();
    /**
     * @brief Select the configured unparked joint goal before entering the motion state.
     * @return Result of GenericState::OnEntry().
     */
    fsm::retval OnEntry() override;
};

}
}

#endif
