#ifndef FER_CTRL_STATE_FAULT_HPP
#define FER_CTRL_STATE_FAULT_HPP

#include "fer_ctrl/states/generic_state.hpp"

namespace fer {
namespace states {

class Fault : public GenericState {
public:
    Fault();
    fsm::retval Execute() override;
};

}
}

#endif
