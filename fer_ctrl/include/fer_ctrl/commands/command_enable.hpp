/** @file command_enable.hpp */

#ifndef FER_CTRL_COMMAND_ENABLE_HPP
#define FER_CTRL_COMMAND_ENABLE_HPP

#include "fer_ctrl/commands/generic_command.hpp"

namespace fer {
namespace commands {

/** @brief Request driver enabling after checking readiness; accept an already enabled idle controller. */
class Enable : public GenericCommand {
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
