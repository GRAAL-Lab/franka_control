/** @file state_disabled.hpp */

#ifndef FER_CTRL_STATE_DISABLED_HPP
#define FER_CTRL_STATE_DISABLED_HPP

#include "fer_ctrl/states/generic_state.hpp"

namespace fer {
namespace states {

/** @brief Inactive state awaiting an explicit enable command. */
class Disabled : public GenericState {
public:
    /** @brief Create the state with its default action and completion behavior. */
    Disabled();
    /**
     * @brief Keep the controller disabled without changing goals.
     * @return FSM execution or queued-transition result.
     */
    fsm::retval Execute() override;
};

}
}

#endif
