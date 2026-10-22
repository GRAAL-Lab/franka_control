/** @file generic_command.hpp */

#ifndef FER_CTRL_GENERIC_COMMAND_HPP
#define FER_CTRL_GENERIC_COMMAND_HPP

#include "fer_ctrl/states/generic_state.hpp"

namespace fer {
namespace commands {

/** @brief FSM command base holding the shared robot state and a copied command request. */
class GenericCommand : public fsm::BaseCommand {
public:
    /** @brief Destroy a command through its base interface. */
    virtual ~GenericCommand() = default;
    /** @brief Controller-owned state assigned before FSM command execution. */
    std::shared_ptr<RobotInfo> robotInfo;

    /** @brief Bind the motion state responsible for validating and storing this command goal. */
    void SetState(const std::shared_ptr<states::GenericState>& state);
    /** @brief Copy the request consumed by the next Execute call. */
    void SetRequest(const CommandRequest& request);

protected:
    std::shared_ptr<states::GenericState> state_;
    CommandRequest request_;
};

}
}

#endif
