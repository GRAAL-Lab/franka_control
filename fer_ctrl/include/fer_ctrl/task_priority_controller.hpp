/** @file task_priority_controller.hpp */

#ifndef FER_CTRL_TASK_PRIORITY_CONTROLLER_HPP
#define FER_CTRL_TASK_PRIORITY_CONTROLLER_HPP

#include "fer_ctrl/ctrl_defines.hpp"
#include "fer_ctrl/commands/generic_command.hpp"
#include "fer_ctrl/states/generic_state.hpp"

namespace fer {

/**
 * @brief ROS-independent task-priority controller and command state machine.
 * @details The caller serializes access and supplies time in seconds from one clock.
 * With isSim enabled, that time also advances the TPIK action transition clock.
 */
class TaskPriorityController {
public:
    /**
     * @brief Load the model, tasks, actions and state machine from configuration.
     * @param configurationFile Path to the kinematic controller libconfig file.
     * @param isSim Use caller-provided simulated time for TPIK transitions when true.
     * @throws std::exception If configuration or task initialization fails.
     */
    explicit TaskPriorityController(const std::string& configurationFile, bool isSim = false);

    /**
     * @brief Validate feedback and update the RML arm model.
     * @param feedback Driver state in the robot base frame.
     * @param receiveTime Receive time in seconds on the controller clock.
     * @return True if accepted; invalid feedback faults the controller.
     */
    bool SetFeedback(const RobotFeedback& feedback, double receiveTime);
    /**
     * @brief Validate and dispatch a command in the current FSM state.
     * @param request Motion or lifecycle command to execute.
     * @param currentTime Current controller clock time in seconds.
     * @return Command acceptance and status; execution may finish in later cycles.
     */
    CommandResult ExecuteCommand(const CommandRequest& request, double currentTime);
    /**
     * @brief Advance the FSM and compute one bounded joint velocity command.
     * @param currentTime Current controller clock time in seconds.
     * @return Velocities in rad/s; zero when control cannot run.
     */
    ControlOutput Run(double currentTime);
    /**
     * @brief Enter the fault state, clear velocity goals and retain the supplied status.
     * @param message Reason requiring explicit recovery.
     */
    void SetFault(const std::string& message);

    /** @brief Return the current FSM state identifier. */
    std::string GetState() const;
    /** @brief Return true when control is requested, including enabling; false while disabling. */
    bool IsEnabled() const;
    /**
     * @brief Check connected feedback against the configured timeout.
     * @param currentTime Current controller clock time in seconds.
     * @return True if feedback age is nonnegative and within feedbackTimeout.
     */
    bool HasFreshFeedback(double currentTime) const;
    /** @brief Return the shared feedback, goals, errors and state status. */
    const RobotInfo& GetRobotInfo() const;
    /** @brief Return the validated kinematic controller settings. */
    const KCLConfiguration& GetConfiguration() const;
    /** @brief Return registered tasks for status inspection. */
    const TasksMap& GetTasksMap() const;
    /** @brief Return the TPIK manager for action and activation status inspection. */
    std::shared_ptr<tpik::ActionManager> GetActionManager() const;

private:
    /** @brief Create the RML model, shared controller state, command handlers and FSM states. */
    void InitializeModelAndCommands();
    /** @brief Create the supported tasks and initialize their joint references. */
    void CreateTasks();
    /**
     * @brief Wire state dependencies, configured actions and permitted commands; enable simulation
     * exceptions.
     */
    void ConfigureStateMachine(libconfig::Config& configuration, bool isSim);
    /** @brief Register allowed state transitions and commands, including simulation-only permissions. */
    void ConfigureStatePermissions(bool isSim);
    /** @brief Update the state and Jacobian of every enabled task. */
    void UpdateTasks();
    /** @brief Clamp output velocities using position margins and configured velocity saturation. */
    void ApplyJointLimits(ControlOutput& output);
    /** @brief Apply an immediate state change and cancel any pending conflicting transition. */
    void ChangeState(const std::string& stateName, const std::string& message);

    std::shared_ptr<KCLConfiguration> configuration_;
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
