#ifndef FER_CTRL_STATE_ENABLING_HPP
#define FER_CTRL_STATE_ENABLING_HPP

#include "fer_ctrl/states/generic_state.hpp"

namespace fer {
namespace states {

class Enabling : public GenericState {
public:
    Enabling();
    fsm::retval Execute() override;
};

}
}

#endif
