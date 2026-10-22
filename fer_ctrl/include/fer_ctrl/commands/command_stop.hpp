/** @file command_stop.hpp */

#ifndef FER_CTRL_COMMAND_STOP_HPP
#define FER_CTRL_COMMAND_STOP_HPP

#include "fer_ctrl/commands/generic_command.hpp"

namespace fer {
namespace commands {

/** @brief Clear velocity goals and request idle, or disabled when enabling is still pending. */
class Stop : public GenericCommand {
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
