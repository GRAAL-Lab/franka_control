/** @file command_move_cartesian.hpp */

#ifndef FER_CTRL_COMMAND_MOVE_CARTESIAN_HPP
#define FER_CTRL_COMMAND_MOVE_CARTESIAN_HPP

#include "fer_ctrl/commands/generic_command.hpp"

namespace fer {
namespace commands {

/** @brief Validate a Cartesian goal and request the Cartesian motion state. */
class MoveCartesian : public GenericCommand {
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
