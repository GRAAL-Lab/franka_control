#ifndef FER_CTRL_COMMAND_DISABLE_HPP
#define FER_CTRL_COMMAND_DISABLE_HPP

#include "fer_ctrl/commands/generic_command.hpp"

namespace fer {
namespace commands {

class Disable : public GenericCommand {
public:
    fsm::retval Execute() override;
};

}
}

#endif
