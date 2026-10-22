/** @file command_park.hpp */

#ifndef FER_CTRL_COMMAND_PARK_HPP
#define FER_CTRL_COMMAND_PARK_HPP

#include "fer_ctrl/commands/generic_command.hpp"

namespace fer {
namespace commands {

/** @brief Request motion to the configured parked posture. */
class Park : public GenericCommand {
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
