/** @file state_move_joints_vel.hpp */

#ifndef FER_CTRL_STATE_MOVE_JOINTS_VEL_HPP
#define FER_CTRL_STATE_MOVE_JOINTS_VEL_HPP

#include "fer_ctrl/states/generic_state.hpp"

namespace fer {
namespace states {

/** @brief Track bounded joint velocities until the command watchdog expires. */
class MoveJointsVel : public GenericState {
public:
    /** @brief Create the state with its default action and completion behavior. */
    MoveJointsVel();
    /**
     * @brief Validate and store single/seven-joint velocities and refresh their receive time.
     * @param request Velocities in rad/s; single-joint indices are zero-based.
     * @return True on acceptance; otherwise shared status explains the rejection.
     */
    bool SetVelocityGoal(const CommandRequest& request);
    /**
     * @brief Update the velocity task and return to idle after velocityCommandTimeout.
     * @return FSM execution or queued-transition result.
     */
    fsm::retval Execute() override;
};

}
}

#endif
