/** @file state_disabling.hpp */

#ifndef FER_CTRL_STATE_DISABLING_HPP
#define FER_CTRL_STATE_DISABLING_HPP

#include "fer_ctrl/states/generic_state.hpp"

namespace fer {
namespace states {

/** @brief Inhibit motion while waiting for fresh feedback confirming that driver control has stopped. */
class Disabling : public GenericState {
public:
    /** @brief Create the state using the Idle action. */
    Disabling();
    /**
     * @brief Enter disabled on fresh stop confirmation, or fault when disableTimeout expires.
     * @details Confirmation requires connected feedback received since entry; missing or stale
     * feedback cannot confirm that the driver has stopped.
     * @return FSM execution or queued-transition result.
     */
    fsm::retval Execute() override;
};

}
}

#endif
