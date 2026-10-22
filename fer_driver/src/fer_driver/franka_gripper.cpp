#include "fer_driver/franka_gripper.hpp"
#include "fer_msgs/fer_defines.hpp"

#include <cmath>

using std::placeholders::_1;
using std::placeholders::_2;

namespace fer {

FrankaGripper::FrankaGripper()
    : Node("gripper_node")
{
    gripper_ = std::make_unique<franka::Gripper>(declare_parameter("robot_ip", "172.16.0.2"));
    callbackGroup_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    gripperStatePub_ = create_publisher<fer_msgs::msg::GripperState>(topicnames::gripper_state, 1);
    commandService_ = create_service<fer_msgs::srv::DriverCommand>(topicnames::gripper_command,
        std::bind(&FrankaGripper::HandleGripperCommand, this, _1, _2), rclcpp::ServicesQoS(), callbackGroup_);
    stateTimer_ = create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&FrankaGripper::PublishState, this), callbackGroup_);
}

void FrankaGripper::HandleGripperCommand(fer_msgs::srv::DriverCommand::Request::SharedPtr request,
    fer_msgs::srv::DriverCommand::Response::SharedPtr response)
{
    if (request->command_type == "stop") {
        try {
            response->success = gripper_->stop();
            response->message = response->success ? "Gripper stopped" : "Gripper stop failed";
        } catch (const std::exception& exception) {
            response->success = false;
            response->message = exception.what();
        }
        return;
    }
    if (request->command_type != "home" && request->command_type != "move"
        && request->command_type != "grasp") {
        response->success = false;
        response->message = "Unsupported gripper command";
        return;
    }
    if (request->command_type != "home"
        && (!std::isfinite(request->width) || request->width < 0.0 || request->width > 0.08
            || !std::isfinite(request->speed) || request->speed <= 0.0 || request->speed > 0.2
            || !std::isfinite(request->force) || request->force < 5.0 || request->force > 70.0)) {
        response->success = false;
        response->message = "Invalid gripper width, speed or force";
        return;
    }
    if (busy_.exchange(true)) {
        response->success = false;
        response->message = "Gripper is busy";
        return;
    }

    try {
        if (request->command_type == "home") {
            response->success = gripper_->homing();
        } else if (request->command_type == "move") {
            response->success = gripper_->move(request->width, request->speed);
        } else {
            response->success = gripper_->grasp(request->width, request->speed, request->force);
        }
        response->message = response->success ? "Gripper command completed" : "Gripper command failed";
    } catch (const std::exception& exception) {
        response->success = false;
        response->message = exception.what();
    }
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        errorMessage_ = response->success ? "" : response->message;
    }
    busy_.store(false);
}

void FrankaGripper::PublishState()
{
    // Preserve the measurement timestamp while a blocking move is in progress.
    if (busy_.load()) {
        std::lock_guard<std::mutex> lock(stateMutex_);
        auto message = lastState_;
        message.busy = true;
        message.error = errorMessage_;
        gripperStatePub_->publish(message);
        return;
    }

    try {
        franka::GripperState state = gripper_->readOnce();
        fer_msgs::msg::GripperState message;
        message.header.stamp = now();
        message.width = state.width;
        message.max_width = state.max_width;
        message.is_grasped = state.is_grasped;
        message.busy = busy_.load();
        {
            std::lock_guard<std::mutex> lock(stateMutex_);
            message.error = errorMessage_;
            lastState_ = message;
        }
        gripperStatePub_->publish(message);
    } catch (const std::exception& exception) {
        std::lock_guard<std::mutex> lock(stateMutex_);
        errorMessage_ = exception.what();
    }
}

}
