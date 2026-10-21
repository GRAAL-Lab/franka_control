#ifndef FER_CTRL_COMMAND_MOVE_CARTESIAN_HPP
#define FER_CTRL_COMMAND_MOVE_CARTESIAN_HPP

#include "fer_ctrl/commands/generic_command.hpp"

namespace fer {
namespace commands {

class MoveCartesian : public GenericCommand {
public:
    fsm::retval Execute() override;
};

}
}

#endif
