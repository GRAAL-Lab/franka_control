/** @file configuration.hpp */

#ifndef FER_CTRL_CONFIGURATION_HPP
#define FER_CTRL_CONFIGURATION_HPP

#include "fer_ctrl/ctrl_defines.hpp"

namespace fer {

/**
 * @brief Load each registered task and validate its activation parameters.
 * @param tasksMap Tasks indexed by their configuration identifiers.
 * @param configuration Parsed controller configuration.
 * @throws std::exception If a task definition is missing or invalid.
 */
void ConfigureTasksFromFile(TasksMap& tasksMap, libconfig::Config& configuration);
/**
 * @brief Register configured priority levels and resolve their task identifiers.
 * @param actionManager Manager receiving the priority levels.
 * @param tasksMap Registered tasks available to each level.
 * @param configuration Parsed controller configuration.
 * @throws std::exception If a level references an unknown task or is invalid.
 */
void ConfigurePriorityLevelsFromFile(const std::shared_ptr<tpik::ActionManager>& actionManager,
    const TasksMap& tasksMap, libconfig::Config& configuration);
/**
 * @brief Register actions from the configured priority-level sequences.
 * @param actionManager Manager containing the referenced priority levels.
 * @param configuration Parsed controller configuration.
 * @throws std::exception If action definitions or level references are invalid.
 */
void ConfigureActionsFromFile(
    const std::shared_ptr<tpik::ActionManager>& actionManager, libconfig::Config& configuration);

}

#endif
