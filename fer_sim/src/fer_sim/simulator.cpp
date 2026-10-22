#include "fer_sim/simulator.hpp"
#include "fer_msgs/conversions.hpp"
#include "fer_msgs/fer_defines.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <chrono>
#include <stdexcept>

using std::placeholders::_1;
using std::placeholders::_2;

namespace fer {

Simulator::Simulator()
    : Node("simulator_node")
    , model_("Panda", declare_parameter("tool_offset", DEFAULT_TOOL_OFFSET))
{
    LoadConfiguration();
    dynamicController_ = std::make_unique<DynamicController>(conf_);
    stopReferenceGenerator_ = std::make_unique<JointReferenceGenerator>(conf_);
    stopReferenceGenerator_->Reset(jointPosition_);
    jointReference_.position = jointPosition_;
    Initialization();
}

void Simulator::LoadConfiguration()
{
    if (!declare_parameter("isSim", true)) {
        throw std::invalid_argument("simulator_node requires isSim=true");
    }
    set_parameter(rclcpp::Parameter("use_sim_time", true));
    simTimeScale_ = declare_parameter("simTimeScale", 1.0);
    if (!std::isfinite(simTimeScale_) || simTimeScale_ <= 0.0) {
        throw std::invalid_argument("simTimeScale must be finite and positive");
    }
    std::string defaultFile = ament_index_cpp::get_package_share_directory("fer_ctrl") + "/conf/dcl_fer.conf";
    conf_.LoadConfiguration(declare_parameter("dynamic_configuration_file", defaultFile));
    loopRate_ = declare_parameter("loop_rate", 1000.0);
    publishRate_ = declare_parameter("publish_rate", 100.0);
    commandTimeout_ = declare_parameter("command_timeout", 0.2);
    baseFrame_ = declare_parameter("base_frame", "panda_link0");

    auto ReadJointParameter = [this](const std::string& name, const JointVector& initial) {
        auto values
            = declare_parameter(name, std::vector<double>(initial.data(), initial.data() + ARM_JOINTS));
        if (values.size() != ARM_JOINTS) {
            throw std::invalid_argument(name + " must contain seven values");
        }
        return JointVector(Eigen::Map<const JointVector>(values.data()));
    };
    jointPosition_ = ReadJointParameter("initial_joints", HomeJointPosition());
    JointVector defaultInertia;
    defaultInertia << 3.0, 3.0, 2.0, 2.0, 0.3, 0.3, 0.2;
    jointInertia_ = ReadJointParameter("joint_inertia", defaultInertia);
    jointDamping_ = ReadJointParameter("joint_damping", JointVector::Constant(0.1));
    massMatrix_ = jointInertia_.asDiagonal();

    if (!JointsWithinLimits(jointPosition_) || !jointInertia_.allFinite() || !jointDamping_.allFinite()
        || (jointInertia_.array() <= 0.0).any() || (jointDamping_.array() < 0.0).any()
        || !std::isfinite(loopRate_) || loopRate_ < 100.0 || loopRate_ > 1000.0
        || !std::isfinite(publishRate_) || publishRate_ < 10.0 || publishRate_ > loopRate_
        || !std::isfinite(commandTimeout_) || commandTimeout_ <= 0.0) {
        throw std::invalid_argument("Invalid simulator posture, inertia, damping, rate or timeout");
    }
}

void Simulator::Initialization()
{
    dynamicControlSub_ = create_subscription<fer_msgs::msg::DynamicControlData>(topicnames::dynamic_control,
        rclcpp::QoS(1).reliable(), std::bind(&Simulator::DynamicControlCB, this, _1));
    stateDataPub_ = create_publisher<fer_msgs::msg::StateData>(
        topicnames::state_data, rclcpp::SensorDataQoS().keep_last(1));
    jointStatePub_ = create_publisher<sensor_msgs::msg::JointState>(
        topicnames::joint_states, rclcpp::SensorDataQoS().keep_last(1));
    gripperStatePub_ = create_publisher<fer_msgs::msg::GripperState>(topicnames::gripper_state, 1);
    driverCommandService_ = create_service<fer_msgs::srv::DriverCommand>(
        topicnames::driver_command, std::bind(&Simulator::DriverCommandsHandler, this, _1, _2));
    gripperCommandService_ = create_service<fer_msgs::srv::DriverCommand>(
        topicnames::gripper_command, std::bind(&Simulator::GripperCommandsHandler, this, _1, _2));
    clockPub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
    const double wallPeriod = 1.0 / (loopRate_ * simTimeScale_);
    if (!std::isfinite(wallPeriod) || wallPeriod < 1e-9) {
        throw std::invalid_argument("simTimeScale produces an invalid timer period");
    }
    runTimer_ = create_wall_timer(
        std::chrono::duration<double>(wallPeriod), std::bind(&Simulator::Run, this));
}

void Simulator::DynamicControlCB(const fer_msgs::msg::DynamicControlData& message)
{
    lastCommandTime_ = simulationTime_.seconds();
    commandEnabled_ = message.control_enabled;
    JointReference reference;
    reference.position = Eigen::Map<const JointVector>(message.joint_positions.data());
    reference.velocity = Eigen::Map<const JointVector>(message.joint_velocities.data());
    reference.acceleration = Eigen::Map<const JointVector>(message.joint_accelerations.data());
    if (!JointsWithinLimits(reference.position) || !reference.velocity.allFinite()
        || !reference.acceleration.allFinite() || message.header.frame_id != baseFrame_
        || reference.velocity.cwiseAbs().maxCoeff() > conf_.maxJointVelocity + 1e-9
        || reference.acceleration.cwiseAbs().maxCoeff() > conf_.maxJointAcceleration + 1e-9) {
        commandEnabled_ = false;
        SetError("Invalid dynamic joint reference");
        return;
    }
    if (commandEnabled_ && !stopping_) {
        jointReference_ = reference;
    }
}

void Simulator::DriverCommandsHandler(fer_msgs::srv::DriverCommand::Request::SharedPtr request,
    fer_msgs::srv::DriverCommand::Response::SharedPtr response)
{
    response->success = true;
    if (request->command_type == "enable" && !hasError_ && !controlActive_) {
        controlRequested_ = true;
        stopping_ = false;
    } else if (request->command_type == "disable") {
        controlRequested_ = false;
    } else if (request->command_type == "recover" && !controlActive_) {
        controlRequested_ = false;
        hasError_ = false;
        errorMessage_.clear();
    } else {
        response->success = false;
        response->message = "Unsupported command or simulator not ready";
        return;
    }
    response->message = "Simulator operation accepted";
}

void Simulator::GripperCommandsHandler(fer_msgs::srv::DriverCommand::Request::SharedPtr request,
    fer_msgs::srv::DriverCommand::Response::SharedPtr response)
{
    response->success = true;
    if (request->command_type == "stop") {
        gripperTarget_ = gripperWidth_;
    } else if (request->command_type == "home") {
        gripperTarget_ = 0.08;
        gripperSpeed_ = 0.05;
    } else if ((request->command_type == "move" || request->command_type == "grasp")
        && std::isfinite(request->width) && request->width >= 0.0 && request->width <= 0.08
        && std::isfinite(request->speed) && request->speed > 0.0 && request->speed <= 0.2
        && std::isfinite(request->force) && request->force >= 5.0 && request->force <= 70.0) {
        gripperTarget_ = request->width;
        gripperSpeed_ = request->speed;
    } else {
        response->success = false;
        response->message = "Invalid gripper command";
        return;
    }
    response->message = "Simulated gripper operation accepted";
}

void Simulator::SetError(const std::string& message)
{
    hasError_ = true;
    controlRequested_ = false;
    errorMessage_ = message;
}

void Simulator::Run()
{
    const auto step = rclcpp::Duration::from_seconds(1.0 / loopRate_);
    simulationTime_ += step;
    const double currentTime = simulationTime_.seconds();
    const double sampleTime = step.seconds();
    bool commandFresh = currentTime - lastCommandTime_ <= commandTimeout_;
    if (controlRequested_ && !controlActive_ && commandEnabled_ && commandFresh && !hasError_) {
        controlActive_ = true;
    }
    if (controlActive_ && !commandFresh) {
        SetError("Dynamic command watchdog expired");
    }
    if (controlActive_ && (!controlRequested_ || !commandEnabled_ || hasError_) && !stopping_) {
        stopping_ = true;
        controlRequested_ = false;
        stopStartTime_ = currentTime;
        stopReferenceGenerator_->SetReference(jointReference_);
    }
    if (controlActive_) {
        const int substeps = static_cast<int>(std::ceil(sampleTime / 0.001));
        for (int index = 0; index < substeps && controlActive_; ++index) {
            RunDynamicControl(sampleTime / substeps);
        }
    }

    double widthError = gripperTarget_ - gripperWidth_;
    gripperWidth_ += std::clamp(widthError, -gripperSpeed_ * sampleTime, gripperSpeed_ * sampleTime);

    rosgraph_msgs::msg::Clock clock;
    clock.clock = simulationTime_;
    clockPub_->publish(clock);
    if (currentTime - lastPublishTime_ >= 1.0 / publishRate_ - 1e-9) {
        PublishState();
        lastPublishTime_ = currentTime;
    }
}

void Simulator::RunDynamicControl(double sampleTime)
{
    if (sampleTime <= 0.0) {
        return;
    }
    if (stopping_) {
        jointReference_ = stopReferenceGenerator_->Run(JointVector::Zero(), jointPosition_, sampleTime);
    }
    JointVector torque;
    if (!dynamicController_->ComputeTorques(jointReference_, jointPosition_, jointVelocity_, massMatrix_,
            JointVector::Zero(), commandedTorque_, sampleTime, torque)) {
        SetError("Invalid dynamic controller output");
        controlActive_ = false;
        jointVelocity_.setZero();
        return;
    }

    commandedTorque_ = torque;
    JointVector acceleration
        = (commandedTorque_ - jointDamping_.cwiseProduct(jointVelocity_)).cwiseQuotient(jointInertia_);
    jointVelocity_ += acceleration * sampleTime;
    jointPosition_ += jointVelocity_ * sampleTime;
    for (std::size_t joint = 0; joint < ARM_JOINTS; ++joint) {
        double limitedPosition
            = std::clamp(jointPosition_[joint], JointLimitsMin()[joint], JointLimitsMax()[joint]);
        if (limitedPosition != jointPosition_[joint]) {
            jointPosition_[joint] = limitedPosition;
            jointVelocity_[joint] = 0.0;
        }
    }

    bool referenceStopped = jointReference_.velocity.cwiseAbs().maxCoeff() < 1e-4
        && jointReference_.acceleration.cwiseAbs().maxCoeff() < 1e-3;
    if (stopping_ && referenceStopped && jointVelocity_.cwiseAbs().maxCoeff() < 0.01
        && simulationTime_.seconds() - stopStartTime_ > 0.1) {
        controlActive_ = false;
        jointVelocity_.setZero();
        commandedTorque_.setZero();
    }
}

void Simulator::PublishState()
{
    model_.JointsPosition(jointPosition_);
    model_.JointsVelocity(jointVelocity_);
    Eigen::TransformationMatrix toolFrame = model_.TransformationMatrix(model_.ToolFrameID());
    JacobianMatrix toolJacobian = model_.Jacobian(model_.ToolFrameID());
    const auto& flangeToTool = model_.FlangeToTool();
    fer_msgs::msg::StateData message;
    message.header.stamp = simulationTime_;
    message.header.frame_id = baseFrame_;
    message.connected = true;
    message.ready = !hasError_;
    message.control_active = controlActive_;
    message.has_error = hasError_;
    message.error = errorMessage_;
    message.robot_mode = controlActive_ ? 2 : 1;
    message.control_command_success_rate = 1.0;
    message.joint_state.header = message.header;
    message.joint_state.name = JointNames();
    message.joint_state.position.assign(jointPosition_.data(), jointPosition_.data() + ARM_JOINTS);
    message.joint_state.velocity.assign(jointVelocity_.data(), jointVelocity_.data() + ARM_JOINTS);
    message.joint_state.effort.assign(commandedTorque_.data(), commandedTorque_.data() + ARM_JOINTS);
    message.tool_pose = PoseToMessage(toolFrame);
    std::copy_n(toolJacobian.data(), 42, message.zero_jacobian.begin());
    std::copy_n(flangeToTool.data(), 16, message.flange_to_tool.begin());
    std::copy_n(massMatrix_.data(), 49, message.mass_matrix.begin());
    std::copy_n(commandedTorque_.data(), ARM_JOINTS, message.commanded_torques.begin());
    stateDataPub_->publish(message);
    jointStatePub_->publish(message.joint_state);

    fer_msgs::msg::GripperState gripperState;
    gripperState.header = message.header;
    gripperState.width = gripperWidth_;
    gripperState.max_width = 0.08;
    gripperState.busy = std::abs(gripperTarget_ - gripperWidth_) > 1e-5;
    gripperState.is_grasped = false;
    gripperStatePub_->publish(gripperState);
}

}
