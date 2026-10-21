#ifndef FER_CTRL_COMMAND_ENABLE_HPP
#define FER_CTRL_COMMAND_ENABLE_HPP

#include "fer_ctrl/commands/generic_command.hpp"

namespace fer {
namespace commands {

class Enable : public GenericCommand {
public:
    fsm::retval Execute() override;
};

}
}

#endif
