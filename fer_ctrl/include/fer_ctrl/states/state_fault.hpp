/** @file state_fault.hpp */

#ifndef FER_CTRL_STATE_FAULT_HPP
#define FER_CTRL_STATE_FAULT_HPP

#include "fer_ctrl/states/generic_state.hpp"

namespace fer {
namespace states {

/** @brief Inactive error state requiring explicit recovery. */
class Fault : public GenericState {
public:
    /** @brief Create the state with its default action and completion behavior. */
    Fault();
    /**
     * @brief Preserve the fault until a lifecycle command changes state.
     * @return FSM execution or queued-transition result.
     */
    fsm::retval Execute() override;
};

}
}

#endif
