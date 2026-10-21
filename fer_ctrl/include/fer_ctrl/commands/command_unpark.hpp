#ifndef FER_CTRL_COMMAND_UNPARK_HPP
#define FER_CTRL_COMMAND_UNPARK_HPP

#include "fer_ctrl/commands/generic_command.hpp"

namespace fer {
namespace commands {

class Unpark : public GenericCommand {
public:
    fsm::retval Execute() override;
};

}
}

#endif
