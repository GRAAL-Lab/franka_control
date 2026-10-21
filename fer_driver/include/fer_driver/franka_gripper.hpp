#ifndef FER_DRIVER_FRANKA_GRIPPER_HPP
#define FER_DRIVER_FRANKA_GRIPPER_HPP

#include "fer_msgs/msg/gripper_state.hpp"
#include "fer_msgs/srv/driver_command.hpp"
#include <franka/gripper.h>
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <mutex>

namespace fer {

class FrankaGripper : public rclcpp::Node {
public:
    FrankaGripper();

private:
    void CommandsHandler(fer_msgs::srv::DriverCommand::Request::SharedPtr request,
        fer_msgs::srv::DriverCommand::Response::SharedPtr response);
    void PublishState();

    std::unique_ptr<franka::Gripper> gripper_;
    std::atomic<bool> busy_ { false };
    std::mutex stateMutex_;
    std::string errorMessage_;
    fer_msgs::msg::GripperState lastState_;
    rclcpp::CallbackGroup::SharedPtr callbackGroup_;
    rclcpp::Publisher<fer_msgs::msg::GripperState>::SharedPtr gripperStatePub_;
    rclcpp::Service<fer_msgs::srv::DriverCommand>::SharedPtr commandService_;
    rclcpp::TimerBase::SharedPtr stateTimer_;
};

}

#endif
