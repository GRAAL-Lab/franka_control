/** @file franka_gripper.hpp */

#ifndef FER_DRIVER_FRANKA_GRIPPER_HPP
#define FER_DRIVER_FRANKA_GRIPPER_HPP

#include "fer_msgs/msg/gripper_state.hpp"
#include "fer_msgs/srv/driver_command.hpp"
#include <franka/gripper.h>
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <mutex>

namespace fer {

/**
 * @brief Hardware gripper service and state publisher using a reentrant callback group.
 * @details An atomic busy flag serializes motion commands; stop can interrupt a blocking move.
 * The state mutex protects the cached ROS state and error message.
 */
class FrankaGripper : public rclcpp::Node {
public:
    /** @brief Connect the gripper and create its command service and wall-clock state timer. */
    FrankaGripper();

private:
    /**
     * @brief Validate and execute a blocking gripper operation; return rejection when a motion is already
     * busy.
     */
    void HandleGripperCommand(fer_msgs::srv::DriverCommand::Request::SharedPtr request,
        fer_msgs::srv::DriverCommand::Response::SharedPtr response);
    /** @brief Publish a fresh measurement, or the cached timestamped state while a motion is busy. */
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
