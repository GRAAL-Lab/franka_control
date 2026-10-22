/** @file kinematic_controller.hpp */

#ifndef FER_CTRL_KINEMATIC_CONTROLLER_HPP
#define FER_CTRL_KINEMATIC_CONTROLLER_HPP

#include "fer_ctrl/task_priority_controller.hpp"
#include "fer_msgs/msg/control_data.hpp"
#include "fer_msgs/msg/state_data.hpp"
#include "fer_msgs/msg/task_status.hpp"
#include "fer_msgs/msg/tpik_action.hpp"
#include "fer_msgs/srv/control_command.hpp"
#include "fer_msgs/srv/driver_command.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

namespace fer {

/** @brief ROS interface for command services, feedback, joint velocity output and task status. */
class KinematicController : public rclcpp::Node {
public:
    /** @brief Load parameters and create the core controller and ROS interfaces using the selected clock. */
    KinematicController();

private:
    using DriverClient = rclcpp::Client<fer_msgs::srv::DriverCommand>;

    /** @brief Outstanding asynchronous driver request, timed in seconds on the node clock. */
    struct PendingDriverRequest {
        DriverClient::SharedPtr client;
        DriverClient::FutureAndRequestId future;
        std::string command;
        double requestTime;
    };

    /** @brief Create subscriptions, publishers, services, TF and ROS-clock timers. */
    void InitializeROSInterfaces();
    /** @brief Validate driver message shape/frame, update feedback and publish the measured tool TF. */
    void HandleStateData(const fer_msgs::msg::StateData& message);
    /** @brief Dispatch arm or gripper requests and report acceptance without blocking on the driver. */
    void HandleControlCommand(fer_msgs::srv::ControlCommand::Request::SharedPtr request,
        fer_msgs::srv::ControlCommand::Response::SharedPtr response);
    /** @brief Validate gripper width/speed/force and queue the corresponding driver request. */
    void HandleGripperCommand(const fer_msgs::srv::ControlCommand::Request& request,
        fer_msgs::srv::ControlCommand::Response& response);
    /** @brief Send a driver service request and retain its future with the current node time. */
    void SendDriverCommand(const DriverClient::SharedPtr& client,
        const std::shared_ptr<fer_msgs::srv::DriverCommand::Request>& request);
    /** @brief Collect driver replies and expire requests after 30 node-clock seconds. */
    void CheckDriverResponses();
    /** @brief Compute and publish one control cycle; publish zero velocities and fault on failure. */
    void Run();
    /** @brief Publish controller state, goal and joint velocities; broadcast the active Cartesian goal. */
    void PublishControl(const ControlOutput& output);
    /** @brief Publish task errors and activation values for the current action. */
    void PublishTasksInfo(const ControlOutput& output);
    /** @brief Publish the current TPIK action and its ordered priority levels. */
    void PublishAction();
    /** @brief Broadcast a child-to-parent RML transform with the current node timestamp. */
    void PublishToTF(const Eigen::TransformationMatrix& frame, const std::string& parentFrame,
        const std::string& childFrame);

    std::unique_ptr<TaskPriorityController> controller_;
    std::string baseFrame_;
    std::string toolFrame_;
    std::string goalFrame_;
    std::vector<PendingDriverRequest> pendingRequests_;
    DriverClient::SharedPtr driverClient_;
    DriverClient::SharedPtr gripperClient_;
    rclcpp::Subscription<fer_msgs::msg::StateData>::SharedPtr stateDataSub_;
    rclcpp::Publisher<fer_msgs::msg::ControlData>::SharedPtr controlDataPub_;
    rclcpp::Publisher<fer_msgs::msg::TaskStatus>::SharedPtr taskStatusPub_;
    rclcpp::Publisher<fer_msgs::msg::TPIKAction>::SharedPtr tpikActionPub_;
    std::unordered_map<std::string, rclcpp::Publisher<fer_msgs::msg::TaskStatus>::SharedPtr> taskPublishers_;
    rclcpp::Service<fer_msgs::srv::ControlCommand>::SharedPtr commandService_;
    rclcpp::TimerBase::SharedPtr runTimer_;
    rclcpp::TimerBase::SharedPtr slowTimer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfPublisher_;
};

}

#endif
