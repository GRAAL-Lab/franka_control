#include "fer_ctrl/configuration.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace fer {
namespace {

double ReadPositive(const libconfig::Setting& setting, const char* name)
{
    double value = setting[name];
    if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument(std::string(setting.getPath()) + "." + name + " must be positive");
    }
    return value;
}

JointVector ReadJoints(const libconfig::Setting& setting, const char* name)
{
    const auto& values = setting[name];
    if (values.getLength() != static_cast<int>(ARM_JOINTS)) {
        throw std::invalid_argument(std::string(name) + " must contain seven values");
    }
    JointVector result;
    for (std::size_t joint = 0; joint < ARM_JOINTS; ++joint) {
        result[joint] = static_cast<double>(values[joint]);
    }
    if (!result.allFinite()) {
        throw std::invalid_argument(std::string(name) + " must contain finite values");
    }
    return result;
}

void ValidateTaskParameters(const libconfig::Setting& task, const std::string& name, int taskSpace)
{
    ReadPositive(task, "gain");
    ReadPositive(task, "saturation");
    static_cast<void>(static_cast<bool>(task["enable"]));
    if (name != ID::tasks::jointsVelocity) {
        int type = task["type"];
        if (type != 0 && type != 1) {
            throw std::invalid_argument(name + ".type must be 0 (equality) or 1 (inequality)");
        }
    }
    if (name == ID::tasks::jointsLimit) {
        const JointVector lowerStart = ReadJoints(task, "activationStartDistanceGreaterThanParams");
        const JointVector upperStart = ReadJoints(task, "activationStartDistanceLessThanParams");
        const JointVector lowerLength = ReadJoints(task, "activationIntervaLengthGreaterThanParams");
        const JointVector upperLength = ReadJoints(task, "activationIntervaLengthLessThanParams");
        if ((lowerStart.array() < 0.0).any() || (upperStart.array() < 0.0).any()
            || (lowerLength.array() <= 0.0).any() || (upperLength.array() <= 0.0).any()
            || ((lowerStart + lowerLength + upperStart + upperLength).array()
                >= (JointLimitsMax() - JointLimitsMin()).array())
                   .any()) {
            throw std::invalid_argument("Invalid JointsLimit activation intervals");
        }
    }
    if (name == ID::tasks::obstacleAvoidance) {
        const auto& bounds = task["greaterThanParams"];
        if (bounds["xmin"].getLength() != taskSpace || bounds["xmax"].getLength() != taskSpace) {
            throw std::invalid_argument("ObstacleAvoidance bounds must contain one distance per plane");
        }
        for (int plane = 0; plane < taskSpace; ++plane) {
            double minimum = bounds["xmin"][plane];
            double maximum = bounds["xmax"][plane];
            if (!std::isfinite(minimum) || !std::isfinite(maximum) || minimum < 0.0 || minimum >= maximum) {
                throw std::invalid_argument("Invalid ObstacleAvoidance distance interval");
            }
        }
    }
}

}

void KCLConfiguration::ConfigureFromFile(libconfig::Config& configuration)
{
    configuration.setAutoConvert(true);
    const auto& root = configuration.getRoot();
    loopRate = ReadPositive(root, "loopRate");
    linearErrorThreshold = ReadPositive(root, "linearErrorThreshold");
    angularErrorThreshold = ReadPositive(root, "angularErrorThreshold");
    jointErrorThreshold = ReadPositive(root, "jointErrorThreshold");
    feedbackTimeout = ReadPositive(root, "feedbackTimeout");
    velocityCommandTimeout = ReadPositive(root, "velocityCommandTimeout");
    moveTimeout = ReadPositive(root, "moveTimeout");
    enableTimeout = ReadPositive(root, "enableTimeout");
    jointLimitMargin = ReadPositive(root, "jointLimitMargin");
    actionTransitionDuration = root["actionTransitionDuration"];
    saturationMin = ReadJoints(root, "saturationMin");
    saturationMax = ReadJoints(root, "saturationMax");
    parkedJointsPos = ReadJoints(root, "parkedJointsPos");
    unparkedJointsPos = ReadJoints(root, "unparkedJointsPos");

    if (loopRate < 10.0 || loopRate > 1000.0 || !std::isfinite(actionTransitionDuration)
        || actionTransitionDuration < 0.0 || (saturationMin.array() >= 0.0).any()
        || (saturationMax.array() <= 0.0).any() || saturationMin.cwiseAbs().maxCoeff() > 2.175
        || saturationMax.maxCoeff() > 2.175 || !JointsWithinLimits(parkedJointsPos, jointLimitMargin)
        || !JointsWithinLimits(unparkedJointsPos, jointLimitMargin)) {
        throw std::invalid_argument("Invalid controller rate, saturation, transition duration or posture");
    }

    const auto& obstacleList = root["obstacles"];
    for (int index = 0; index < obstacleList.getLength(); ++index) {
        const auto& setting = obstacleList[index];
        PlaneObstacleConfiguration obstacle;
        obstacle.name = setting["name"].c_str();
        const auto& normal = setting["normal"];
        if (normal.getLength() != 3) {
            throw std::invalid_argument("Obstacle normal must contain three values");
        }
        obstacle.normal = Eigen::Vector3d(normal[0], normal[1], normal[2]);
        obstacle.offset = setting["offset"];
        if (!obstacle.normal.allFinite() || obstacle.normal.norm() < 1e-9
            || !std::isfinite(obstacle.offset)) {
            throw std::invalid_argument("Invalid obstacle plane");
        }
        obstacles.push_back(obstacle);
    }
    if (obstacles.empty()) {
        throw std::invalid_argument(
            "Configure at least one obstacle plane; disable its task to omit avoidance");
    }
}

void ConfigureTasksFromFile(TasksMap& tasksMap, libconfig::Config& configuration)
{
    const auto& taskSettings = configuration.getRoot()["tasks"];
    for (const auto& task : tasksMap) {
        if (!taskSettings.exists(task.first)) {
            throw std::invalid_argument("Missing task configuration: " + task.first);
        }
        ValidateTaskParameters(taskSettings[task.first.c_str()], task.first, task.second.task->TaskSpace());
        if (!task.second.task->ConfigFromFile(configuration)) {
            throw std::invalid_argument("Failed to configure task: " + task.first);
        }
    }
    for (int index = 0; index < taskSettings.getLength(); ++index) {
        const std::string taskName = taskSettings[index].getName();
        if (tasksMap.count(taskName) == 0) {
            throw std::invalid_argument("Unknown task in configuration: " + taskName);
        }
    }
}

void ConfigurePriorityLevelsFromFile(const std::shared_ptr<tpik::ActionManager>& actionManager,
    const TasksMap& tasksMap, libconfig::Config& configuration)
{
    const auto& levels = configuration.getRoot()["priorityLevels"];
    std::vector<std::string> hierarchy;
    std::set<std::string> names;
    for (int index = 0; index < levels.getLength(); ++index) {
        const auto& level = levels[index];
        std::string name = level["name"].c_str();
        if (name.empty() || !names.insert(name).second || level["tasks"].getLength() == 0) {
            throw std::invalid_argument("Empty/duplicate priority level or empty task list: " + name);
        }
        rml::RegularizationData regularization;
        regularization.params.lambda = ReadPositive(level, "lambda");
        regularization.params.threshold = ReadPositive(level, "threshold");
        actionManager->AddPriorityLevelWithRegularization(name, regularization);
        std::set<std::string> levelTasks;
        const auto& taskNames = level["tasks"];
        for (int taskIndex = 0; taskIndex < taskNames.getLength(); ++taskIndex) {
            std::string taskName = taskNames[taskIndex].c_str();
            if (tasksMap.count(taskName) == 0 || !levelTasks.insert(taskName).second) {
                throw std::invalid_argument("Unknown/duplicate task in " + name + ": " + taskName);
            }
            actionManager->AddTaskToPriorityLevel(tasksMap.at(taskName).task, name);
        }
        hierarchy.push_back(name);
    }
    if (hierarchy.empty()) {
        throw std::invalid_argument("priorityLevels must not be empty");
    }
    actionManager->SetUnifiedHierarchy(hierarchy);
}

void ConfigureActionsFromFile(
    const std::shared_ptr<tpik::ActionManager>& actionManager, libconfig::Config& configuration)
{
    const auto& actions = configuration.getRoot()["actions"];
    std::set<std::string> names;
    const auto& hierarchy = actionManager->GetHierarchy();
    for (int index = 0; index < actions.getLength(); ++index) {
        const auto& action = actions[index];
        std::string name = action["name"].c_str();
        if (name.empty() || !names.insert(name).second) {
            throw std::invalid_argument("Empty/duplicate action: " + name);
        }
        std::vector<std::string> levels;
        int previousPosition = -1;
        const auto& levelNames = action["levels"];
        for (int levelIndex = 0; levelIndex < levelNames.getLength(); ++levelIndex) {
            std::string levelName = levelNames[levelIndex].c_str();
            auto found = std::find_if(hierarchy.begin(), hierarchy.end(),
                [&levelName](const auto& level) { return level->ID() == levelName; });
            int position = static_cast<int>(std::distance(hierarchy.begin(), found));
            if (found == hierarchy.end() || position <= previousPosition) {
                throw std::invalid_argument(
                    "Unknown, duplicate or out-of-order priority level in " + name + ": " + levelName);
            }
            previousPosition = position;
            levels.push_back(levelName);
        }
        actionManager->AddAction(name, levels);
    }
    for (const auto& required :
        { "Idle", "ParkUnpark", "MoveJointsPosition", "MoveJointsVelocity", "MoveTool" }) {
        if (names.count(required) == 0) {
            throw std::invalid_argument(std::string("Missing action: ") + required);
        }
    }
}

}
