#include "fer_ctrl/dynamic_control_node.hpp"
#include "fer_msgs/conversions.hpp"
#include "fer_msgs/fer_defines.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>

using std::placeholders::_1;

namespace fer {

DynamicControlNode::DynamicControlNode()
    : Node("dynamic_control_node")
{
    std::string defaultFile = ament_index_cpp::get_package_share_directory("fer_ctrl") + "/conf/dcl_fer.conf";
    std::string configurationFile = declare_parameter("configuration_file", defaultFile);
    conf_.LoadConfiguration(configurationFile);
    baseFrame_ = declare_parameter("base_frame", "panda_link0");
    referenceGenerator_ = std::make_unique<JointReferenceGenerator>(conf_);
    referenceGenerator_->Reset(jointPosition_);

    stateDataSub_ = create_subscription<fer_msgs::msg::StateData>(topicnames::state_data,
        rclcpp::SensorDataQoS().keep_last(1), std::bind(&DynamicControlNode::StateDataCB, this, _1));
    controlDataSub_ = create_subscription<fer_msgs::msg::ControlData>(topicnames::ctrl_data,
        rclcpp::QoS(1).reliable(), std::bind(&DynamicControlNode::ControlDataCB, this, _1));
    dynamicControlPub_ = create_publisher<fer_msgs::msg::DynamicControlData>(
        topicnames::dynamic_control, rclcpp::QoS(1).reliable());
    lastRunTime_ = SteadyTime();
    runTimer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / conf_.loopRate), std::bind(&DynamicControlNode::Run, this));
    RCLCPP_INFO(get_logger(), "[DCL] %s reference generator loaded from %s", conf_.controlMode.c_str(),
        configurationFile.c_str());
}

void DynamicControlNode::StateDataCB(const fer_msgs::msg::StateData& message)
{
    receivingFeedback_ = false;
    if (!message.connected || message.header.frame_id != baseFrame_
        || message.joint_state.name != JointNames() || message.joint_state.position.size() != ARM_JOINTS) {
        return;
    }
    JointVector jointPosition = Eigen::Map<const JointVector>(message.joint_state.position.data());
    if (!jointPosition.allFinite()) {
        return;
    }
    jointPosition_ = jointPosition;
    controlActive_ = message.control_active;
    driverError_ = message.has_error;
    lastFeedbackTime_ = SteadyTime();
    receivingFeedback_ = true;
}

void DynamicControlNode::ControlDataCB(const fer_msgs::msg::ControlData& message)
{
    receivingCommands_ = false;
    JointVector requestedVelocity = Eigen::Map<const JointVector>(message.joint_velocities.data());
    if (!requestedVelocity.allFinite()
        || requestedVelocity.cwiseAbs().maxCoeff() > conf_.maxJointVelocity + 1e-9) {
        return;
    }
    requestedVelocity_ = requestedVelocity;
    controlRequested_ = message.control_enabled;
    controllerState_ = message.ctrl_state;
    lastCommandTime_ = SteadyTime();
    receivingCommands_ = true;
}

void DynamicControlNode::Run()
{
    double currentTime = SteadyTime();
    double sampleTime = currentTime - lastRunTime_;
    lastRunTime_ = currentTime;
    bool feedbackFresh = receivingFeedback_ && currentTime - lastFeedbackTime_ <= conf_.feedbackTimeout;
    bool commandFresh = receivingCommands_ && currentTime - lastCommandTime_ <= conf_.commandTimeout;
    bool enabled = feedbackFresh && commandFresh && controlRequested_ && !driverError_;

    if (!enabled || !controlActive_) {
        referenceGenerator_->Reset(jointPosition_);
    } else {
        referenceGenerator_->Run(requestedVelocity_, jointPosition_, sampleTime);
    }

    std::string status = "Dynamic reference tracking";
    if (!feedbackFresh) {
        status = "Waiting for fresh robot feedback";
    } else if (!commandFresh) {
        status = "Kinematic command timeout";
    } else if (!enabled) {
        status = "Dynamic control disabled";
    } else if (!controlActive_) {
        status = "Waiting for torque control acknowledgement";
    }
    PublishControl(enabled, status);
}

void DynamicControlNode::PublishControl(bool enabled, const std::string& status)
{
    fer_msgs::msg::DynamicControlData message;
    message.header.stamp = now();
    message.header.frame_id = baseFrame_;
    message.control_enabled = enabled;
    message.ctrl_state = controllerState_;
    message.status = status;
    const JointReference& reference = referenceGenerator_->GetReference();
    std::copy_n(reference.position.data(), ARM_JOINTS, message.joint_positions.begin());
    std::copy_n(reference.velocity.data(), ARM_JOINTS, message.joint_velocities.begin());
    std::copy_n(reference.acceleration.data(), ARM_JOINTS, message.joint_accelerations.begin());
    dynamicControlPub_->publish(message);
}

}
