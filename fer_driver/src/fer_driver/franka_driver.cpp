#include "fer_driver/franka_driver.hpp"
#include "fer_msgs/conversions.hpp"
#include "fer_msgs/fer_defines.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <chrono>

using std::placeholders::_1;
using std::placeholders::_2;

namespace fer {

FrankaDriver::FrankaDriver()
    : Node("driver_node")
{
    LoadConfiguration();
    dynamicController_ = std::make_unique<DynamicController>(conf_);
    stopReferenceGenerator_ = std::make_unique<JointReferenceGenerator>(conf_);
    Initialization();
    controlThread_ = std::thread(&FrankaDriver::Run, this);
}

FrankaDriver::~FrankaDriver()
{
    running_.store(false);
    enableRequested_.store(false);
    if (controlThread_.joinable()) {
        controlThread_.join();
    }
}

void FrankaDriver::LoadConfiguration()
{
    std::string defaultFile = ament_index_cpp::get_package_share_directory("fer_ctrl") + "/conf/dcl_fer.conf";
    conf_.LoadConfiguration(declare_parameter("dynamic_configuration_file", defaultFile));
    robotAddress_ = declare_parameter("robot_ip", "172.16.0.2");
    baseFrame_ = declare_parameter("base_frame", "panda_link0");
    commandTimeout_ = declare_parameter("command_timeout", 0.2);
    publishRate_ = declare_parameter("publish_rate", 100.0);
    enforceRealtime_ = declare_parameter("enforce_realtime", true);
    if (!std::isfinite(commandTimeout_) || commandTimeout_ <= 0.0 || !std::isfinite(publishRate_)
        || publishRate_ < 10.0 || publishRate_ > 1000.0) {
        throw std::invalid_argument("Invalid driver command timeout or feedback rate");
    }
}

void FrankaDriver::Initialization()
{
    dynamicControlSub_ = create_subscription<fer_msgs::msg::DynamicControlData>(topicnames::dynamic_control,
        rclcpp::QoS(1).reliable(), std::bind(&FrankaDriver::DynamicControlCB, this, _1));
    stateDataPub_ = create_publisher<fer_msgs::msg::StateData>(
        topicnames::state_data, rclcpp::SensorDataQoS().keep_last(1));
    jointStatePub_ = create_publisher<sensor_msgs::msg::JointState>(
        topicnames::joint_states, rclcpp::SensorDataQoS().keep_last(1));
    commandService_ = create_service<fer_msgs::srv::DriverCommand>(
        topicnames::driver_command, std::bind(&FrankaDriver::CommandsHandler, this, _1, _2));
    publishTimer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / publishRate_), std::bind(&FrankaDriver::PublishFeedback, this));
}

void FrankaDriver::DynamicControlCB(const fer_msgs::msg::DynamicControlData& message)
{
    ControlReference command;
    command.receivedTime = SteadyTime();
    command.enabled = message.control_enabled;
    JointVector position = Eigen::Map<const JointVector>(message.joint_positions.data());
    JointVector velocity = Eigen::Map<const JointVector>(message.joint_velocities.data());
    JointVector acceleration = Eigen::Map<const JointVector>(message.joint_accelerations.data());
    if (!position.allFinite() || !velocity.allFinite() || !acceleration.allFinite()
        || !JointsWithinLimits(position) || velocity.cwiseAbs().maxCoeff() > conf_.maxJointVelocity + 1e-9
        || acceleration.cwiseAbs().maxCoeff() > conf_.maxJointAcceleration + 1e-9
        || message.header.frame_id != baseFrame_) {
        SetError("Invalid dynamic joint reference");
        command.enabled = false;
    } else {
        command.position = message.joint_positions;
        command.velocity = message.joint_velocities;
        command.acceleration = message.joint_accelerations;
    }
    commandBuffer_.Publish(command);
}

void FrankaDriver::CommandsHandler(fer_msgs::srv::DriverCommand::Request::SharedPtr request,
    fer_msgs::srv::DriverCommand::Response::SharedPtr response)
{
    response->success = true;
    if (request->command_type == "disable") {
        enableRequested_.store(false);
        response->message = "Disable requested";
    } else if (request->command_type == "enable") {
        if (hasError_.load() || recovering_.load() || controlActive_.load() || !ready_.load()) {
            response->success = false;
            response->message = "Driver is not ready, already controlling, or recovery is required";
        } else {
            enableRequested_.store(true);
            response->message = "Enable accepted; waiting for fresh dynamic references";
        }
    } else if (request->command_type == "recover") {
        if (!connected_.load()) {
            response->success = false;
            response->message = "No robot connection; restart the driver";
        } else if (controlActive_.load() || enableRequested_.load() || recovering_.exchange(true)) {
            response->success = false;
            response->message = "Disable and wait for control to stop before recovery";
        } else {
            recoverRequested_.store(true);
            response->message = "Recovery queued";
        }
    } else {
        response->success = false;
        response->message = "Unsupported arm driver command";
    }
}

void FrankaDriver::SetError(const std::string& message)
{
    hasError_.store(true);
    enableRequested_.store(false);
    RCLCPP_ERROR(get_logger(), "[Driver] %s", message.c_str());
    std::lock_guard<std::mutex> lock(errorMutex_);
    errorMessage_ = message;
}

void FrankaDriver::Recover()
{
    try {
        robot_->automaticErrorRecovery();
        hasError_.store(false);
        std::lock_guard<std::mutex> lock(errorMutex_);
        errorMessage_.clear();
    } catch (const std::exception& exception) {
        SetError(exception.what());
    }
    recovering_.store(false);
}

void FrankaDriver::Run()
{
    FeedbackSnapshot snapshot;
    try {
        franka::RealtimeConfig realtime
            = enforceRealtime_ ? franka::RealtimeConfig::kEnforce : franka::RealtimeConfig::kIgnore;
        robot_ = std::make_unique<franka::Robot>(robotAddress_, realtime);
        model_ = std::make_unique<franka::Model>(robot_->loadModel());

        while (running_.load()) {
            if (recoverRequested_.exchange(false)) {
                Recover();
            }
            commandBuffer_.Consume(controlReference_);
            bool commandFresh = SteadyTime() - controlReference_.receivedTime <= commandTimeout_;
            if (enableRequested_.load() && !hasError_.load() && controlReference_.enabled && commandFresh) {
                RunControlLoop();
            } else {
                snapshot = ReadSnapshot(robot_->readOnce(), false);
                feedbackBuffer_.Publish(snapshot);
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        }
    } catch (const std::exception& exception) {
        SetError(exception.what());
        connected_.store(false);
        ready_.store(false);
        controlActive_.store(false);
        snapshot.connected = false;
        snapshot.ready = false;
        snapshot.controlActive = false;
        snapshot.hasError = true;
        feedbackBuffer_.Publish(snapshot);
    }
}

void FrankaDriver::RunControlLoop()
{
    stopping_ = false;
    watchdogExpired_ = false;
    invalidDynamics_ = false;
    stopTimedOut_ = false;
    jointReference_.position = Eigen::Map<const JointVector>(controlReference_.position.data());
    jointReference_.velocity = Eigen::Map<const JointVector>(controlReference_.velocity.data());
    jointReference_.acceleration = Eigen::Map<const JointVector>(controlReference_.acceleration.data());
    controlActive_.store(true);
    try {
        robot_->control(std::bind(&FrankaDriver::TorqueControlCB, this, _1, _2), true, 100.0);
        if (watchdogExpired_) {
            SetError("Dynamic command watchdog expired; recover before enabling");
        } else if (invalidDynamics_) {
            SetError("Invalid dynamic model, torque state or controller output");
        } else if (stopTimedOut_) {
            SetError("Torque control stop did not settle before timeout");
        }
    } catch (const std::exception& exception) {
        SetError(exception.what());
    }
    controlActive_.store(false);
    enableRequested_.store(false);
}

franka::Torques FrankaDriver::TorqueControlCB(const franka::RobotState& state, franka::Duration period)
{
    commandBuffer_.Consume(controlReference_);
    double currentTime = SteadyTime();
    double sampleTime = period.toSec();
    bool expired = currentTime - controlReference_.receivedTime > commandTimeout_;
    bool stopRequested = !running_.load() || !enableRequested_.load() || !controlReference_.enabled || expired
        || hasError_.load();
    watchdogExpired_ |= expired;
    if (stopRequested && !stopping_) {
        stopping_ = true;
        stopStartTime_ = currentTime;
        enableRequested_.store(false);
        stopReferenceGenerator_->SetReference(jointReference_);
    }

    JointVector jointPosition = Eigen::Map<const JointVector>(state.q.data());
    JointVector jointVelocity = Eigen::Map<const JointVector>(state.dq.data());
    if (stopping_) {
        jointReference_ = stopReferenceGenerator_->Run(JointVector::Zero(), jointPosition, sampleTime);
    } else {
        jointReference_.position = Eigen::Map<const JointVector>(controlReference_.position.data());
        jointReference_.velocity = Eigen::Map<const JointVector>(controlReference_.velocity.data());
        jointReference_.acceleration = Eigen::Map<const JointVector>(controlReference_.acceleration.data());
    }

    FeedbackSnapshot snapshot = ReadSnapshot(state, true);
    feedbackBuffer_.Publish(snapshot);
    if (sampleTime <= 0.0) {
        return franka::Torques(state.tau_J_d);
    }

    JointVector torque;
    bool valid = dynamicController_->ComputeTorques(jointReference_, jointPosition, jointVelocity,
        Eigen::Map<const JointMatrix>(snapshot.massMatrix.data()),
        Eigen::Map<const JointVector>(snapshot.coriolis.data()),
        Eigen::Map<const JointVector>(state.tau_J_d.data()), sampleTime, torque);
    if (!valid) {
        invalidDynamics_ = true;
        return franka::MotionFinished(franka::Torques(state.tau_J_d));
    }

    std::array<double, ARM_JOINTS> torqueCommand;
    std::copy_n(torque.data(), ARM_JOINTS, torqueCommand.begin());
    franka::Torques output(torqueCommand);
    bool referenceStopped = jointReference_.velocity.cwiseAbs().maxCoeff() < 1e-4
        && jointReference_.acceleration.cwiseAbs().maxCoeff() < 1e-3;
    if (stopping_ && referenceStopped && jointVelocity.cwiseAbs().maxCoeff() < 0.01
        && currentTime - stopStartTime_ > 0.1) {
        return franka::MotionFinished(output);
    }
    if (stopping_ && currentTime - stopStartTime_ > 3.0) {
        stopTimedOut_ = true;
        return franka::MotionFinished(output);
    }
    return output;
}

FrankaDriver::FeedbackSnapshot FrankaDriver::ReadSnapshot(const franka::RobotState& state, bool controlActive)
{
    FeedbackSnapshot snapshot;
    snapshot.position = state.q;
    snapshot.velocity = state.dq;
    snapshot.measuredTorque = state.tau_J;
    snapshot.commandedTorque = state.tau_J_d;
    snapshot.toolFrame = state.O_T_EE;
    snapshot.flangeToTool = state.F_T_EE;
    snapshot.toolJacobian = model_->zeroJacobian(franka::Frame::kEndEffector, state);
    snapshot.massMatrix = model_->mass(state);
    snapshot.coriolis = model_->coriolis(state);
    snapshot.connected = true;
    snapshot.controlActive = controlActive;
    snapshot.hasError = static_cast<bool>(state.current_errors) || hasError_.load();
    snapshot.ready = !snapshot.hasError
        && (state.robot_mode == franka::RobotMode::kIdle || state.robot_mode == franka::RobotMode::kMove);
    snapshot.robotMode = static_cast<uint8_t>(state.robot_mode);
    snapshot.successRate = state.control_command_success_rate;
    connected_.store(true);
    ready_.store(snapshot.ready);
    return snapshot;
}

void FrankaDriver::PublishFeedback()
{
    FeedbackSnapshot snapshot;
    if (!feedbackBuffer_.Consume(snapshot)) {
        return;
    }

    fer_msgs::msg::StateData message;
    message.header.stamp = now();
    message.header.frame_id = baseFrame_;
    message.connected = snapshot.connected;
    message.ready = snapshot.ready;
    message.control_active = snapshot.controlActive;
    message.has_error = snapshot.hasError;
    message.robot_mode = snapshot.robotMode;
    message.control_command_success_rate = snapshot.successRate;
    {
        std::lock_guard<std::mutex> lock(errorMutex_);
        message.error = errorMessage_;
    }
    message.joint_state.header = message.header;
    message.joint_state.name = JointNames();
    message.joint_state.position.assign(snapshot.position.begin(), snapshot.position.end());
    message.joint_state.velocity.assign(snapshot.velocity.begin(), snapshot.velocity.end());
    message.joint_state.effort.assign(snapshot.measuredTorque.begin(), snapshot.measuredTorque.end());
    Pose toolFrame = Pose::Identity();
    if (snapshot.connected) {
        toolFrame.matrix() = Eigen::Map<const Eigen::Matrix4d>(snapshot.toolFrame.data());
    }
    message.tool_pose = PoseToMessage(toolFrame);
    message.flange_to_tool = snapshot.flangeToTool;
    message.zero_jacobian = snapshot.toolJacobian;
    message.mass_matrix = snapshot.massMatrix;
    message.coriolis = snapshot.coriolis;
    message.commanded_torques = snapshot.commandedTorque;
    stateDataPub_->publish(message);
    jointStatePub_->publish(message.joint_state);
}

}
