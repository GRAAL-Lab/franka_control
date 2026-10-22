#include "fer_ctrl/task_priority_controller.hpp"
#include "fer_ctrl/configuration.hpp"
#include "fer_ctrl/tasks/joints_limit.hpp"
#include "fer_ctrl/tasks/obstacle_avoidance.hpp"

#include "fer_ctrl/commands/command_disable.hpp"
#include "fer_ctrl/commands/command_enable.hpp"
#include "fer_ctrl/commands/command_stop.hpp"
#include "fer_ctrl/commands/command_park.hpp"
#include "fer_ctrl/commands/command_unpark.hpp"
#include "fer_ctrl/commands/command_move_joints_pos.hpp"
#include "fer_ctrl/commands/command_move_joints_vel.hpp"
#include "fer_ctrl/commands/command_move_cartesian.hpp"
#include "fer_ctrl/states/state_disabled.hpp"
#include "fer_ctrl/states/state_enabling.hpp"
#include "fer_ctrl/states/state_idle.hpp"
#include "fer_ctrl/states/state_parked.hpp"
#include "fer_ctrl/states/state_parking.hpp"
#include "fer_ctrl/states/state_unparking.hpp"
#include "fer_ctrl/states/state_move_joints_pos.hpp"
#include "fer_ctrl/states/state_move_joints_vel.hpp"
#include "fer_ctrl/states/state_move_cartesian.hpp"
#include "fer_ctrl/states/state_fault.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fer {

TaskPriorityController::TaskPriorityController(const std::string& configurationFile, bool isSim)
{
    libconfig::Config configuration;
    configuration.readFile(configurationFile.c_str());
    configuration_ = std::make_shared<KCLConfiguration>();
    configuration_->ConfigureFromFile(configuration);
    InitializeModelAndCommands();
    CreateTasks();
    ConfigureTasksFromFile(tasksMap_, configuration);
    ConfigurePriorityLevelsFromFile(actionManager_, tasksMap_, configuration);
    ConfigureActionsFromFile(actionManager_, configuration);

    actionManager_->IsSimulation(isSim);
    actionManager_->Time(0);
    actionManager_->SetTransitionDuration(configuration_->actionTransitionDuration * 1000.0);
    iCat_ = std::make_shared<tpik::iCAT>(ARM_JOINTS);
    iCat_->SetSaturation(configuration_->saturationMin, configuration_->saturationMax);
    solver_ = std::make_shared<tpik::Solver>(actionManager_, iCat_);
    ConfigureStateMachine(configuration, isSim);
}

void TaskPriorityController::InitializeModelAndCommands()
{
    robotInfo_ = std::make_shared<RobotInfo>();
    armModel_ = std::make_shared<FrankaArmModel>();
    Eigen::TransformationMatrix identity;
    robotModel_ = std::make_shared<rml::RobotModel>(identity, "PandaBase");
    robotModel_->LoadArm(armModel_, identity);
    actionManager_ = std::make_shared<tpik::ActionManager>();

    statesMap_ = { { ID::states::disabled, std::make_shared<states::Disabled>() },
        { ID::states::enabling, std::make_shared<states::Enabling>() },
        { ID::states::idle, std::make_shared<states::Idle>() },
        { ID::states::parked, std::make_shared<states::Parked>() },
        { ID::states::parking, std::make_shared<states::Parking>() },
        { ID::states::unparking, std::make_shared<states::Unparking>() },
        { ID::states::move_joints_pos, std::make_shared<states::MoveJointsPos>() },
        { ID::states::move_joints_vel, std::make_shared<states::MoveJointsVel>() },
        { ID::states::move_cartesian, std::make_shared<states::MoveCartesian>() },
        { ID::states::fault, std::make_shared<states::Fault>() } };
    commandsMap_ = { { "enable", std::make_shared<commands::Enable>() },
        { "disable", std::make_shared<commands::Disable>() },
        { "recover", std::make_shared<commands::Disable>() }, { "stop", std::make_shared<commands::Stop>() },
        { "park", std::make_shared<commands::Park>() }, { "unpark", std::make_shared<commands::Unpark>() },
        { "move_joints_pos", std::make_shared<commands::MoveJointsPos>() },
        { "move_joints_vel", std::make_shared<commands::MoveJointsVel>() },
        { "move_cartesian", std::make_shared<commands::MoveCartesian>() } };
}

void TaskPriorityController::CreateTasks()
{
    const std::string armID = armModel_->ID();
    tasksMap_[ID::tasks::jointsLimit].task = std::make_shared<tasks::JointsLimit>(robotModel_, armID);
    tasksMap_[ID::tasks::jointsPosition].task
        = std::make_shared<ikcl::JointsPosition>(ID::tasks::jointsPosition, robotModel_, armID);
    tasksMap_[ID::tasks::jointsVelocity].task
        = std::make_shared<ikcl::JointsVelocity>(ID::tasks::jointsVelocity, robotModel_, armID);
    tasksMap_[ID::tasks::jointsPosture].task
        = std::make_shared<ikcl::JointsPosition>(ID::tasks::jointsPosture, robotModel_, armID);
    tasksMap_[ID::tasks::toolDistance].task
        = std::make_shared<ikcl::CartesianDistance>(ID::tasks::toolDistance, robotModel_, robotInfo_->toolID);
    tasksMap_[ID::tasks::toolOrientation].task = std::make_shared<ikcl::CartesianOrientation>(
        ID::tasks::toolOrientation, robotModel_, robotInfo_->toolID);

    tasksMap_[ID::tasks::obstacleAvoidance].task = std::make_shared<tasks::ObstacleAvoidance>(
        robotModel_, robotInfo_->toolID, configuration_->obstacles);

    auto position
        = std::dynamic_pointer_cast<ikcl::JointsPosition>(tasksMap_.at(ID::tasks::jointsPosition).task);
    auto posture
        = std::dynamic_pointer_cast<ikcl::JointsPosition>(tasksMap_.at(ID::tasks::jointsPosture).task);
    auto velocity
        = std::dynamic_pointer_cast<ikcl::JointsVelocity>(tasksMap_.at(ID::tasks::jointsVelocity).task);
    position->Reference() = HomeJointPosition();
    posture->Reference() = configuration_->unparkedJointsPos;
    velocity->SetReferenceRate(JointVector::Zero());
}

void TaskPriorityController::ConfigureStateMachine(libconfig::Config& configuration, bool isSim)
{
    for (auto& [stateName, state] : statesMap_) {
        state->robotInfo = robotInfo_;
        state->configuration = configuration_;
        state->armModel = armModel_;
        state->robotModel = robotModel_;
        state->actionManager = actionManager_;
        state->tasksMap = tasksMap_;
        state->SetFSM(&controllerFsm_);
        state->ConfigureStateFromFile(configuration);
        controllerFsm_.AddState(stateName, state.get());
    }
    for (auto& [commandName, command] : commandsMap_) {
        command->robotInfo = robotInfo_;
        command->SetFSM(&controllerFsm_);
        controllerFsm_.AddCommand(commandName, command.get());
    }
    commandsMap_.at("move_joints_pos")->SetState(statesMap_.at(ID::states::move_joints_pos));
    commandsMap_.at("move_joints_vel")->SetState(statesMap_.at(ID::states::move_joints_vel));
    commandsMap_.at("move_cartesian")->SetState(statesMap_.at(ID::states::move_cartesian));

    ConfigureStatePermissions(isSim);
    controllerFsm_.SetInitState(ID::states::disabled);
}

void TaskPriorityController::ConfigureStatePermissions(bool isSim)
{
    for (const auto& currentState : statesMap_) {
        for (const auto& nextState : statesMap_) {
            if (nextState.first != currentState.first) {
                controllerFsm_.EnableTransition(currentState.first, nextState.first, true);
            }
        }
        for (const auto& command : { "disable", "stop", "recover" }) {
            controllerFsm_.EnableCommandInState(currentState.first, command, true);
        }
    }
    controllerFsm_.EnableCommandInState(ID::states::disabled, "enable", true);
    for (const auto& command : commandsMap_) {
        controllerFsm_.EnableCommandInState(ID::states::idle, command.first, true);
    }
    controllerFsm_.EnableCommandInState(ID::states::parked, "unpark", true);
    const std::vector<std::string> motionStates
        = { ID::states::move_joints_pos, ID::states::move_joints_vel, ID::states::move_cartesian };
    for (const auto& state : motionStates) {
        for (const auto& command : { "park", "move_joints_pos", "move_joints_vel", "move_cartesian" }) {
            controllerFsm_.EnableCommandInState(state, command, true);
        }
    }
    if (isSim) {
        controllerFsm_.EnableCommandInState(ID::states::parked, "move_joints_pos", true);
    }
}

bool TaskPriorityController::SetFeedback(const RobotFeedback& feedback, double receiveTime)
{
    const Eigen::RotationMatrix rotation = feedback.toolFrame.RotationMatrix();
    if (!std::isfinite(receiveTime) || !feedback.jointPosition.allFinite()
        || !feedback.jointVelocity.allFinite() || !feedback.toolFrame.allFinite()
        || !feedback.flangeToTool.allFinite() || !feedback.toolJacobian.allFinite()
        || (rotation.transpose() * rotation - Eigen::RotationMatrix::Identity()).norm() > 1e-5
        || std::abs(rotation.determinant() - 1.0) > 1e-5) {
        robotInfo_->receivingFeedback = false;
        SetFault("Invalid robot feedback");
        return false;
    }
    robotInfo_->feedback = feedback;
    robotInfo_->lastFeedbackTime = receiveTime;
    robotInfo_->receivingFeedback = true;
    armModel_->JointsPosition(feedback.jointPosition);
    armModel_->JointsVelocity(feedback.jointVelocity);
    armModel_->SetToolFrame(feedback.flangeToTool);
    return true;
}

bool TaskPriorityController::HasFreshFeedback(double currentTime) const
{
    const double feedbackAge = currentTime - robotInfo_->lastFeedbackTime;
    return robotInfo_->receivingFeedback && robotInfo_->feedback.connected && feedbackAge >= 0.0
        && feedbackAge <= configuration_->feedbackTimeout;
}

CommandResult TaskPriorityController::ExecuteCommand(const CommandRequest& request, double currentTime)
{
    robotInfo_->currentTime = currentTime;
    actionManager_->Time(static_cast<long>(currentTime * 1000.0));
    std::string commandName = request.commandType;
    if (commandName == "move_joint_i_pos") {
        commandName = "move_joints_pos";
    } else if (commandName == "move_joint_i_vel") {
        commandName = "move_joints_vel";
    }
    if (commandsMap_.count(commandName) == 0) {
        return { false, "Unsupported command: " + request.commandType };
    }
    const bool canInterruptMotion
        = commandName == "disable" || commandName == "stop" || commandName == "recover";
    if (!canInterruptMotion && !HasFreshFeedback(currentTime)) {
        return { false, "No fresh robot feedback" };
    }
    if (!controllerFsm_.CommandAllowed(commandName)) {
        return { false, "Command is not allowed in state " + GetState() };
    }
    if (commandName == "recover" && robotInfo_->feedback.controlActive) {
        return { false, "Disable and wait for control to stop before recovery" };
    }
    if (!canInterruptMotion && commandName != "enable"
        && (!robotInfo_->feedback.controlActive || robotInfo_->feedback.hasError)) {
        return { false, "Enable the controller before moving" };
    }

    if (canInterruptMotion && controllerFsm_.GetNextStateName() != GetState()) {
        // SetNextState(current) is a no-op in FSM. Reset its pending target so
        // stop/disable/recover can cancel a transition before the timer applies it.
        controllerFsm_.SetInitState(GetState());
    }
    auto command = commandsMap_.at(commandName);
    command->SetRequest(request);
    fsm::retval result = controllerFsm_.ExecuteCommand(commandName);
    return { result == fsm::ok, robotInfo_->status };
}

void TaskPriorityController::UpdateTasks()
{
    for (const auto& [taskName, taskInfo] : tasksMap_) {
        if (taskInfo.task->Enabled()) {
            taskInfo.task->Update();
        }
    }
}

void TaskPriorityController::ApplyJointLimits(ControlOutput& output)
{
    for (std::size_t joint = 0; joint < ARM_JOINTS; ++joint) {
        const double position = robotInfo_->feedback.jointPosition[joint];
        const double margin = configuration_->jointLimitMargin;
        // Reduce outward velocity near a limit while always allowing a joint to retreat.
        double lower = std::min(0.0, -(position - JointLimitsMin()[joint] - margin));
        double upper = std::max(0.0, JointLimitsMax()[joint] - margin - position);
        lower = std::max(lower, configuration_->saturationMin[joint]);
        upper = std::min(upper, configuration_->saturationMax[joint]);
        double limitedVelocity = std::clamp(output.jointVelocity[joint], lower, upper);
        output.jointLimitActive |= limitedVelocity != output.jointVelocity[joint];
        output.jointVelocity[joint] = limitedVelocity;
    }
}

ControlOutput TaskPriorityController::Run(double currentTime)
{
    ControlOutput output;
    robotInfo_->currentTime = currentTime;
    actionManager_->Time(static_cast<long>(currentTime * 1000.0));
    controllerFsm_.SwitchState();
    controllerFsm_.ProcessEventQueue();
    if (!HasFreshFeedback(currentTime)) {
        if (IsEnabled()) {
            ChangeState(ID::states::disabled, "Feedback timeout; enable again after reconnecting");
        }
        return output;
    }
    if (robotInfo_->feedback.hasError && IsEnabled()) {
        SetFault("Driver reported an error; recover explicitly");
        return output;
    }
    if (IsEnabled() && GetState() != ID::states::enabling && !robotInfo_->feedback.controlActive) {
        ChangeState(ID::states::disabled, "Driver stopped; enable again to resume");
        return output;
    }

    robotInfo_->jointError = 0.0;
    robotInfo_->linearError = 0.0;
    robotInfo_->angularError = 0.0;
    controllerFsm_.ExecuteState();
    UpdateTasks();
    const std::string state = GetState();
    if (IsEnabled() && state != ID::states::idle && state != ID::states::parked
        && state != ID::states::enabling) {
        output.jointVelocity = solver_->ComputeVelocities();
        if (!output.jointVelocity.allFinite()) {
            SetFault("Task-priority solver returned nonfinite velocities");
            output.jointVelocity.setZero();
        }
        ApplyJointLimits(output);
    } else {
        actionManager_->ComputeActionTransitionActivation();
    }
    output.jointError = robotInfo_->jointError;
    output.linearError = robotInfo_->linearError;
    output.angularError = robotInfo_->angularError;
    return output;
}

void TaskPriorityController::ChangeState(const std::string& stateName, const std::string& message)
{
    robotInfo_->status = message;
    robotInfo_->velocityGoal.setZero();
    if (stateName == GetState() && controllerFsm_.GetNextStateName() != stateName) {
        controllerFsm_.SetInitState(stateName);
    }
    controllerFsm_.SetNextState(stateName);
    controllerFsm_.SwitchState();
}

void TaskPriorityController::SetFault(const std::string& message)
{
    ChangeState(ID::states::fault, message);
}

std::string TaskPriorityController::GetState() const
{
    return controllerFsm_.GetCurrentStateName();
}

bool TaskPriorityController::IsEnabled() const
{
    return GetState() != ID::states::disabled && GetState() != ID::states::fault;
}

const RobotInfo& TaskPriorityController::GetRobotInfo() const
{
    return *robotInfo_;
}

const KCLConfiguration& TaskPriorityController::GetConfiguration() const
{
    return *configuration_;
}

const TasksMap& TaskPriorityController::GetTasksMap() const
{
    return tasksMap_;
}

std::shared_ptr<tpik::ActionManager> TaskPriorityController::GetActionManager() const
{
    return actionManager_;
}

}
