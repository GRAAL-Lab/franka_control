#ifndef FER_CTRL_STATE_IDLE_HPP
#define FER_CTRL_STATE_IDLE_HPP

#include "fer_ctrl/states/generic_state.hpp"

namespace fer {
namespace states {

class Idle : public GenericState {
public:
    Idle();
    fsm::retval Execute() override;
};

}
}

#endif
