#ifndef FER_CTRL_TASK_JOINTS_LIMIT_HPP
#define FER_CTRL_TASK_JOINTS_LIMIT_HPP

#include "fer_ctrl/ctrl_defines.hpp"
#include <ikcl/ikcl.h>

namespace fer {
namespace tasks {

// Honor the enable flag with IKCL versions whose JointsLimit loader omits it.
class JointsLimit : public ikcl::JointsLimit {
public:
    JointsLimit(const std::shared_ptr<rml::RobotModel>& robotModel, const std::string& armID);
    bool ConfigFromFile(libconfig::Config& configuration) override;
};

}
}

#endif
