#include "fer_ctrl/states/state_disabled.hpp"

namespace fer {
namespace states {

Disabled::Disabled()
    : GenericState(ID::states::disabled)
{
}

fsm::retval Disabled::Execute()
{
    return fsm::ok;
}

}
}
