/** @file command_disable.hpp */

#ifndef FER_CTRL_COMMAND_DISABLE_HPP
#define FER_CTRL_COMMAND_DISABLE_HPP

#include "fer_ctrl/commands/generic_command.hpp"

namespace fer {
namespace commands {

/** @brief Request the disabled state for disable or recovery and clear velocity goals. */
class Disable : public GenericCommand {
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
