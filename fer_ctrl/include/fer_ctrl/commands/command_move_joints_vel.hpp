#ifndef FER_CTRL_COMMAND_MOVE_JOINTS_VEL_HPP
#define FER_CTRL_COMMAND_MOVE_JOINTS_VEL_HPP

#include "fer_ctrl/commands/generic_command.hpp"

namespace fer {
namespace commands {

class MoveJointsVel : public GenericCommand {
public:
    fsm::retval Execute() override;
};

}
}

#endif
