/** @file command_move_joints_vel.hpp */

#ifndef FER_CTRL_COMMAND_MOVE_JOINTS_VEL_HPP
#define FER_CTRL_COMMAND_MOVE_JOINTS_VEL_HPP

#include "fer_ctrl/commands/generic_command.hpp"

namespace fer {
namespace commands {

/** @brief Validate joint velocities and refresh the velocity-command watchdog. */
class MoveJointsVel : public GenericCommand {
public:
    /**
     * @brief Execute the stored request using the bound FSM and shared robot state.
     * @return fsm::ok on acceptance, or the validation/transition failure code.
     */
    fsm::retval Execute() override;
};

}
}

#endif
