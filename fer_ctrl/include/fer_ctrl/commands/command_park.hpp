#ifndef FER_CTRL_COMMAND_PARK_HPP
#define FER_CTRL_COMMAND_PARK_HPP

#include "fer_ctrl/commands/generic_command.hpp"

namespace fer {
namespace commands {

class Park : public GenericCommand {
public:
    fsm::retval Execute() override;
};

}
}

#endif
