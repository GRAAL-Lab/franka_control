#include "fer_ctrl/tasks/joints_limit.hpp"

namespace fer {
namespace tasks {

JointsLimit::JointsLimit(const std::shared_ptr<rml::RobotModel>& robotModel, const std::string& armID)
    : ikcl::JointsLimit(ID::tasks::jointsLimit, robotModel, armID)
{
}

bool JointsLimit::ConfigFromFile(libconfig::Config& configuration)
{
    bool configured = ikcl::JointsLimit::ConfigFromFile(configuration);
    enabled_ = taskParameter_.taskEnable;
    return configured;
}

}
}
