#include "fer_ctrl/states/state_fault.hpp"

namespace fer {
namespace states {

Fault::Fault()
    : GenericState(ID::states::fault)
{
}

fsm::retval Fault::Execute()
{
    return fsm::ok;
}

}
}
