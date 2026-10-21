#ifndef FER_CTRL_CONFIGURATION_HPP
#define FER_CTRL_CONFIGURATION_HPP

#include "fer_ctrl/ctrl_defines.hpp"

namespace fer {

void ConfigureTasksFromFile(TasksMap& tasksMap, libconfig::Config& configuration);
void ConfigurePriorityLevelsFromFile(const std::shared_ptr<tpik::ActionManager>& actionManager,
    const TasksMap& tasksMap, libconfig::Config& configuration);
void ConfigureActionsFromFile(
    const std::shared_ptr<tpik::ActionManager>& actionManager, libconfig::Config& configuration);

}

#endif
