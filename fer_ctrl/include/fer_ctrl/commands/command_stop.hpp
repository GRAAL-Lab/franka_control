#ifndef FER_CTRL_COMMAND_STOP_HPP
#define FER_CTRL_COMMAND_STOP_HPP

#include "fer_ctrl/commands/generic_command.hpp"

namespace fer {
namespace commands {

class Stop : public GenericCommand {
public:
    fsm::retval Execute() override;
};

}
}

#endif
