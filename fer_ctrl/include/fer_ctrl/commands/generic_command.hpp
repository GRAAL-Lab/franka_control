#ifndef FER_CTRL_GENERIC_COMMAND_HPP
#define FER_CTRL_GENERIC_COMMAND_HPP

#include "fer_ctrl/states/generic_state.hpp"

namespace fer {
namespace commands {

class GenericCommand : public fsm::BaseCommand {
public:
    virtual ~GenericCommand() = default;
    std::shared_ptr<RobotInfo> robotInfo;

    void SetState(const std::shared_ptr<states::GenericState>& state);
    void SetRequest(const CommandRequest& request);

protected:
    std::shared_ptr<states::GenericState> state_;
    CommandRequest request_;
};

}
}

#endif
