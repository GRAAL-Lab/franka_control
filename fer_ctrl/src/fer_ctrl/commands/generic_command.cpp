#include "fer_ctrl/commands/generic_command.hpp"

namespace fer {
namespace commands {

void GenericCommand::SetState(const std::shared_ptr<states::GenericState>& state)
{
    state_ = state;
}

void GenericCommand::SetRequest(const CommandRequest& request)
{
    request_ = request;
}

}
}
