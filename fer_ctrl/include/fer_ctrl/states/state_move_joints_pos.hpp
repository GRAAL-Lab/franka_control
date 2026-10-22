/** @file state_move_joints_pos.hpp */

#ifndef FER_CTRL_STATE_MOVE_JOINTS_POS_HPP
#define FER_CTRL_STATE_MOVE_JOINTS_POS_HPP

#include "fer_ctrl/states/generic_state.hpp"

namespace fer {
namespace states {

/** @brief Track a joint posture with position and settling thresholds. */
class MoveJointsPos : public GenericState {
public:
    /** @brief Create the state with its default action and completion behavior. */
    MoveJointsPos();
    /**
     * @brief Validate and store an absolute or relative single/seven-joint position goal.
     * @param request Setpoints in radians; single-joint indices are zero-based.
     * @return True on acceptance; otherwise shared status explains the rejection.
     */
    bool SetJointsGoal(const CommandRequest& request);
    /**
     * @brief Update the position task and finish on convergence or moveTimeout.
     * @return FSM execution or queued-transition result.
     */
    fsm::retval Execute() override;

protected:
    /**
     * @brief Create a joint-position state reused by parking and unparking.
     * @param stateName FSM identifier.
     * @param actionName Default TPIK action.
     * @param completionState State requested when the joint goal settles.
     */
    MoveJointsPos(
        const std::string& stateName, const std::string& actionName, const std::string& completionState);
    std::string completionState_;
};

}
}

#endif
