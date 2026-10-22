#ifndef FER_CTRL_TASK_PRIORITY_CONTROLLER_HPP
#define FER_CTRL_TASK_PRIORITY_CONTROLLER_HPP

#include "fer_ctrl/ctrl_defines.hpp"
#include "fer_ctrl/commands/generic_command.hpp"
#include "fer_ctrl/states/generic_state.hpp"

namespace fer {

class TaskPriorityController {
public:
    explicit TaskPriorityController(const std::string& configurationFile, bool isSim = false);

    bool SetFeedback(const RobotFeedback& feedback, double receiveTime);
    CommandResult ExecuteCommand(const CommandRequest& request, double currentTime);
    ControlOutput Run(double currentTime);
    void SetFault(const std::string& message);

    std::string GetState() const;
    bool IsEnabled() const;
    bool HasFreshFeedback(double currentTime) const;
    const RobotInfo& GetRobotInfo() const;
    const KCLConfiguration& GetConfiguration() const;
    const TasksMap& GetTasksMap() const;
    std::shared_ptr<tpik::ActionManager> GetActionManager() const;

private:
    void Initialization();
    void SetUpTasks();
    void SetUpFSM(libconfig::Config& configuration, bool isSim);
    void UpdateTasks();
    void ApplyJointLimits(ControlOutput& output);
    void ChangeState(const std::string& stateName, const std::string& message);

    std::shared_ptr<KCLConfiguration> conf_;
    std::shared_ptr<RobotInfo> robotInfo_;
    std::shared_ptr<FrankaArmModel> armModel_;
    std::shared_ptr<rml::RobotModel> robotModel_;
    std::shared_ptr<tpik::ActionManager> actionManager_;
    std::shared_ptr<tpik::iCAT> iCat_;
    std::shared_ptr<tpik::Solver> solver_;

    fsm::FSM controllerFsm_;
    TasksMap tasksMap_;
    std::unordered_map<std::string, std::shared_ptr<states::GenericState>> statesMap_;
    std::unordered_map<std::string, std::shared_ptr<commands::GenericCommand>> commandsMap_;
};

}

#endif
