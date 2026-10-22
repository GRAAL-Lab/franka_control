/** @file joints_limit.hpp */

#ifndef FER_CTRL_TASK_JOINTS_LIMIT_HPP
#define FER_CTRL_TASK_JOINTS_LIMIT_HPP

#include "fer_ctrl/ctrl_defines.hpp"
#include <ikcl/ikcl.h>

namespace fer {
namespace tasks {

/** @brief Joint-limit task that also honors taskEnable with older IKCL configuration loaders. */
class JointsLimit : public ikcl::JointsLimit {
public:
    /** @brief Create the joint-limit task for the selected RML arm identifier. */
    JointsLimit(const std::shared_ptr<rml::RobotModel>& robotModel, const std::string& armID);
    /**
     * @brief Load IKCL parameters and explicitly apply the configured task enable flag.
     * @param configuration Parsed controller configuration.
     * @return Result from the underlying IKCL configuration loader.
     */
    bool ConfigFromFile(libconfig::Config& configuration) override;
};

}
}

#endif
