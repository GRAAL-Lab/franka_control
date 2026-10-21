#ifndef FER_CTRL_STATE_PARKED_HPP
#define FER_CTRL_STATE_PARKED_HPP

#include "fer_ctrl/states/generic_state.hpp"

namespace fer {
namespace states {

class Parked : public GenericState {
public:
    Parked();
    fsm::retval Execute() override;
};

}
}

#endif
