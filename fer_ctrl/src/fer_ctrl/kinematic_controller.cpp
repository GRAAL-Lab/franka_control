#include "fer_ctrl/kinematic_controller.hpp"
#include "fer_msgs/conversions.hpp"
#include "fer_msgs/fer_defines.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <future>
#include <geometry_msgs/msg/transform_stamped.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

namespace fer {

KinematicController::KinematicController()
    : Node("kinematic_control_node")
{
    std::string defaultFile = ament_index_cpp::get_package_share_directory("fer_ctrl") + "/conf/kcl_fer.conf";
    std::string configurationFile = declare_parameter("configuration_file", defaultFile);
    baseFrame_ = declare_parameter("base_frame", "panda_link0");
    toolFrame_ = declare_parameter("tool_frame", "fer_tool");
    goalFrame_ = declare_parameter("goal_frame", "goal");
    bool simulatedDriver = declare_parameter("simulated_driver", false);
    controller_ = std::make_unique<TaskPriorityController>(configurationFile, simulatedDriver);
    Initialization();
    RCLCPP_INFO(
        get_logger(), "[KCL] Loaded tasks, priority levels and actions from %s", configurationFile.c_str());
}

void KinematicController::Initialization()
{
    stateDataSub_ = create_subscription<fer_msgs::msg::StateData>(topicnames::state_data,
        rclcpp::SensorDataQoS().keep_last(1), std::bind(&KinematicController::StateDataCB, this, _1));
    controlDataPub_
        = create_publisher<fer_msgs::msg::ControlData>(topicnames::ctrl_data, rclcpp::QoS(1).reliable());
    taskStatusPub_ = create_publisher<fer_msgs::msg::TaskStatus>(topicnames::task_status, 1);
    tpikActionPub_ = create_publisher<fer_msgs::msg::TPIKAction>(topicnames::tpik_action, 1);
    for (const auto& task : controller_->GetTasksMap()) {
        taskPublishers_[task.first] = create_publisher<fer_msgs::msg::TaskStatus>("tasks/" + task.first, 1);
    }
    driverClient_ = create_client<fer_msgs::srv::DriverCommand>(topicnames::driver_command);
    gripperClient_ = create_client<fer_msgs::srv::DriverCommand>(topicnames::gripper_command);
    commandService_ = create_service<fer_msgs::srv::ControlCommand>(
        topicnames::control_command, std::bind(&KinematicController::CommandsHandler, this, _1, _2));
    tfPublisher_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    double sampleTime = 1.0 / controller_->GetConfiguration().loopRate;
    runTimer_ = create_wall_timer(
        std::chrono::duration<double>(sampleTime), std::bind(&KinematicController::Run, this));
    slowTimer_ = create_wall_timer(
        std::chrono::milliseconds(200), std::bind(&KinematicController::PublishAction, this));
}

void KinematicController::StateDataCB(const fer_msgs::msg::StateData& message)
{
    try {
        if (message.joint_state.position.size() != ARM_JOINTS
            || message.joint_state.velocity.size() != ARM_JOINTS || message.joint_state.name != JointNames()
            || message.header.frame_id != baseFrame_) {
            throw std::invalid_argument(
                "Feedback must contain Panda joints 1..7 in the configured base frame");
        }
        RobotFeedback feedback;
        feedback.jointPosition = Eigen::Map<const JointVector>(message.joint_state.position.data());
        feedback.jointVelocity = Eigen::Map<const JointVector>(message.joint_state.velocity.data());
        feedback.toolFrame = MessageToPose(message.tool_pose);
        feedback.flangeToTool.matrix() = Eigen::Map<const Eigen::Matrix4d>(message.flange_to_tool.data());
        feedback.toolJacobian = Eigen::Map<const JacobianMatrix>(message.zero_jacobian.data());
        feedback.connected = message.connected;
        feedback.ready = message.ready;
        feedback.controlActive = message.control_active;
        feedback.hasError = message.has_error;
        if (!controller_->SetFeedback(feedback, SteadyTime()) || !feedback.connected) {
            return;
        }

        PublishToTF(feedback.toolFrame, baseFrame_, toolFrame_);
    } catch (const std::exception& exception) {
        controller_->SetFault(exception.what());
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "[KCL] %s", exception.what());
    }
}

void KinematicController::CommandsHandler(fer_msgs::srv::ControlCommand::Request::SharedPtr request,
    fer_msgs::srv::ControlCommand::Response::SharedPtr response)
{
    if (request->command_type.compare(0, 8, "gripper_") == 0) {
        HandleGripperCommand(*request, *response);
        return;
    }

    bool driverOperation = request->command_type == "enable" || request->command_type == "disable"
        || request->command_type == "recover";
    if (request->command_type == "enable" && controller_->GetState() == ID::states::idle
        && controller_->GetRobotInfo().feedback.controlActive) {
        driverOperation = false;
    }
    if (driverOperation && request->command_type != "disable" && !driverClient_->service_is_ready()) {
        response->success = false;
        response->res = "Arm driver service is unavailable";
        return;
    }

    CommandRequest command;
    command.commandType = request->command_type;
    command.moveType = request->move_type;
    command.jointSetpoint = request->joint_setpoint;
    command.jointIndex = request->joint_index;
    command.targetPosition = Eigen::Map<const Eigen::Vector3d>(request->target_position.data());
    command.targetOrientation = Eigen::Map<const Eigen::Vector3d>(request->target_orientation.data());
    command.frameType = request->frame_type;
    CommandResult result = controller_->ExecuteCommand(command, SteadyTime());
    response->success = result.success;
    response->res = result.message;
    if (!result.success) {
        return;
    }

    if (driverOperation && driverClient_->service_is_ready()) {
        auto driverRequest = std::make_shared<fer_msgs::srv::DriverCommand::Request>();
        driverRequest->command_type = request->command_type;
        SendDriverCommand(driverClient_, driverRequest);
        response->res += "; driver request queued";
    }
}

void KinematicController::HandleGripperCommand(
    const fer_msgs::srv::ControlCommand::Request& request, fer_msgs::srv::ControlCommand::Response& response)
{
    const std::string operation = request.command_type.substr(8);
    if (operation != "move" && operation != "grasp" && operation != "home" && operation != "stop") {
        response.success = false;
        response.res = "Unsupported gripper command";
        return;
    }
    if (!gripperClient_->service_is_ready()
        || (operation != "stop" && !controller_->HasFreshFeedback(SteadyTime()))) {
        response.success = false;
        response.res = "Gripper service or robot feedback is unavailable";
        return;
    }
    if ((operation == "move" || operation == "grasp")
        && (!std::isfinite(request.gripper_setpoint) || request.gripper_setpoint < 0.0
            || request.gripper_setpoint > 0.08 || !std::isfinite(request.gripper_speed)
            || request.gripper_speed <= 0.0 || request.gripper_speed > 0.2
            || !std::isfinite(request.grasp_force) || request.grasp_force < 5.0
            || request.grasp_force > 70.0)) {
        response.success = false;
        response.res = "Use width 0..0.08 m, speed (0,0.2] m/s and force 5..70 N";
        return;
    }

    auto driverRequest = std::make_shared<fer_msgs::srv::DriverCommand::Request>();
    driverRequest->command_type = operation;
    driverRequest->width = request.gripper_setpoint;
    driverRequest->speed = request.gripper_speed;
    driverRequest->force = request.grasp_force;
    SendDriverCommand(gripperClient_, driverRequest);
    response.success = true;
    response.res = "Gripper request queued";
}

void KinematicController::SendDriverCommand(const DriverClient::SharedPtr& client,
    const std::shared_ptr<fer_msgs::srv::DriverCommand::Request>& request)
{
    pendingRequests_.push_back(
        { client, client->async_send_request(request), request->command_type, SteadyTime() });
}

void KinematicController::CheckDriverResponses()
{
    for (auto pending = pendingRequests_.begin(); pending != pendingRequests_.end();) {
        bool completed = false;
        if (pending->future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            auto response = pending->future.get();
            if (!response->success) {
                RCLCPP_ERROR(
                    get_logger(), "[KCL] Driver %s: %s", pending->command.c_str(), response->message.c_str());
                if (pending->client == driverClient_ && pending->command == "enable"
                    && controller_->GetState() == ID::states::enabling) {
                    controller_->SetFault(response->message);
                }
            }
            completed = true;
        } else if (SteadyTime() - pending->requestTime > 30.0) {
            pending->client->remove_pending_request(pending->future);
            RCLCPP_ERROR(get_logger(), "[KCL] Driver %s timed out", pending->command.c_str());
            if (pending->client == driverClient_ && controller_->GetState() == ID::states::enabling) {
                controller_->SetFault("Driver service timed out");
            }
            completed = true;
        }

        if (completed) {
            pending = pendingRequests_.erase(pending);
        } else {
            ++pending;
        }
    }
}

void KinematicController::Run()
{
    try {
        ControlOutput output = controller_->Run(SteadyTime());
        CheckDriverResponses();
        PublishControl(output);
        PublishTasksInfo(output);
    } catch (const std::exception& exception) {
        controller_->SetFault(exception.what());
        PublishControl(ControlOutput());
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "[KCL] %s", exception.what());
    }
}

void KinematicController::PublishControl(const ControlOutput& output)
{
    fer_msgs::msg::ControlData message;
    message.header.stamp = now();
    message.header.frame_id = baseFrame_;
    message.ctrl_state = controller_->GetState();
    message.control_enabled = controller_->IsEnabled();
    message.status = controller_->GetRobotInfo().status;
    message.target_pose = PoseToMessage(controller_->GetRobotInfo().cartesianGoalFrame);
    std::copy_n(output.jointVelocity.data(), ARM_JOINTS, message.joint_velocities.begin());
    controlDataPub_->publish(message);
    if (message.ctrl_state == ID::states::move_cartesian) {
        PublishToTF(controller_->GetRobotInfo().cartesianGoalFrame, baseFrame_, goalFrame_);
    }
}

void KinematicController::PublishToTF(
    const Pose& frame, const std::string& parentFrame, const std::string& childFrame)
{
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = parentFrame;
    transform.child_frame_id = childFrame;
    transform.transform.translation.x = frame.translation().x();
    transform.transform.translation.y = frame.translation().y();
    transform.transform.translation.z = frame.translation().z();
    transform.transform.rotation = PoseToMessage(frame).orientation;
    tfPublisher_->sendTransform(transform);
}

void KinematicController::PublishTasksInfo(const ControlOutput& output)
{
    fer_msgs::msg::TaskStatus summary;
    summary.header.stamp = now();
    summary.action = controller_->GetActionManager()->CurrentActionID();
    summary.joint_error = output.jointError;
    summary.linear_error = output.linearError;
    summary.angular_error = output.angularError;
    summary.joint_limit_active = output.jointLimitActive;
    taskStatusPub_->publish(summary);

    auto actionManager = controller_->GetActionManager();
    auto activeLevels = actionManager->GetAction(actionManager->CurrentActionID())->PriorityLevels();
    for (const auto& entry : controller_->GetTasksMap()) {
        fer_msgs::msg::TaskStatus message = summary;
        message.id = entry.first;
        message.enabled = entry.second.task->Enabled();
        for (const auto& level : activeLevels) {
            for (const auto& task : level->Level()) {
                message.in_current_action |= task->ID() == entry.first;
            }
        }
        const auto& internal = entry.second.task->InternalActivationFunction();
        const auto& external = entry.second.task->ExternalActivationFunction();
        const auto& reference = entry.second.task->ReferenceRate();
        for (int index = 0; index < internal.rows(); ++index) {
            message.internal_activation_function.push_back(internal(index, index));
            message.external_activation_function.push_back(external(index, index));
        }
        message.reference_rate.assign(reference.data(), reference.data() + reference.size());
        taskPublishers_.at(entry.first)->publish(message);
    }
}

void KinematicController::PublishAction()
{
    fer_msgs::msg::TPIKAction message;
    message.header.stamp = now();
    auto actionManager = controller_->GetActionManager();
    message.id = actionManager->CurrentActionID();
    for (const auto& level : actionManager->GetAction(message.id)->PriorityLevels()) {
        fer_msgs::msg::TPIKPriorityLevel priorityLevel;
        priorityLevel.id = level->ID();
        for (const auto& task : level->Level()) {
            priorityLevel.tasks_id.push_back(task->ID());
        }
        message.priority_levels.push_back(priorityLevel);
    }
    tpikActionPub_->publish(message);
}

}
