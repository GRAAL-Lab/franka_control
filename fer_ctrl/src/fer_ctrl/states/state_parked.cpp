#include "fer_ctrl/states/state_parked.hpp"

namespace fer {
namespace states {

Parked::Parked()
    : GenericState(ID::states::parked)
{
}

fsm::retval Parked::Execute()
{
    return fsm::ok;
}

}
}
