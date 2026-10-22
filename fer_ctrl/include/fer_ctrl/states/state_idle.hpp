/** @file state_idle.hpp */

#ifndef FER_CTRL_STATE_IDLE_HPP
#define FER_CTRL_STATE_IDLE_HPP

#include "fer_ctrl/states/generic_state.hpp"

namespace fer {
namespace states {

/** @brief Enabled state with zero requested joint velocity. */
class Idle : public GenericState {
public:
    /** @brief Create the state with its default action and completion behavior. */
    Idle();
    /**
     * @brief Clear the velocity goal while awaiting the next motion command.
     * @return FSM execution or queued-transition result.
     */
    fsm::retval Execute() override;
};

}
}

#endif
