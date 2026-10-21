#ifndef FER_CTRL_STATE_DISABLED_HPP
#define FER_CTRL_STATE_DISABLED_HPP

#include "fer_ctrl/states/generic_state.hpp"

namespace fer {
namespace states {

class Disabled : public GenericState {
public:
    Disabled();
    fsm::retval Execute() override;
};

}
}

#endif
