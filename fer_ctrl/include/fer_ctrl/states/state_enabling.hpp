/** @file state_enabling.hpp */

#ifndef FER_CTRL_STATE_ENABLING_HPP
#define FER_CTRL_STATE_ENABLING_HPP

#include "fer_ctrl/states/generic_state.hpp"

namespace fer {
namespace states {

/** @brief Wait for torque-control acknowledgement from the driver. */
class Enabling : public GenericState {
public:
    /** @brief Create the state with its default action and completion behavior. */
    Enabling();
    /**
     * @brief Enter idle on acknowledgement, or fault when enableTimeout expires.
     * @return FSM execution or queued-transition result.
     */
    fsm::retval Execute() override;
};

}
}

#endif
