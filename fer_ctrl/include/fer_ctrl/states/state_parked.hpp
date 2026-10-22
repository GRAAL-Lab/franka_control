/** @file state_parked.hpp */

#ifndef FER_CTRL_STATE_PARKED_HPP
#define FER_CTRL_STATE_PARKED_HPP

#include "fer_ctrl/states/generic_state.hpp"

namespace fer {
namespace states {

/** @brief Enabled state reached after the configured parking motion. */
class Parked : public GenericState {
public:
    /** @brief Create the state with its default action and completion behavior. */
    Parked();
    /**
     * @brief Remain parked while awaiting an allowed command.
     * @return FSM execution or queued-transition result.
     */
    fsm::retval Execute() override;
};

}
}

#endif
