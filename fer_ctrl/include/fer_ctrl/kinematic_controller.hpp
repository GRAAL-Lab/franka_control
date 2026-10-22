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

class KinematicController : public rclcpp::Node {
public:
    KinematicController();

private:
    using DriverClient = rclcpp::Client<fer_msgs::srv::DriverCommand>;

    struct PendingDriverRequest {
        DriverClient::SharedPtr client;
        DriverClient::FutureAndRequestId future;
        std::string command;
        double requestTime;
    };

    void Initialization();
    void StateDataCB(const fer_msgs::msg::StateData& message);
    void CommandsHandler(fer_msgs::srv::ControlCommand::Request::SharedPtr request,
        fer_msgs::srv::ControlCommand::Response::SharedPtr response);
    void HandleGripperCommand(const fer_msgs::srv::ControlCommand::Request& request,
        fer_msgs::srv::ControlCommand::Response& response);
    void SendDriverCommand(const DriverClient::SharedPtr& client,
        const std::shared_ptr<fer_msgs::srv::DriverCommand::Request>& request);
    void CheckDriverResponses();
    void Run();
    void PublishControl(const ControlOutput& output);
    void PublishTasksInfo(const ControlOutput& output);
    void PublishAction();
    void PublishToTF(
        const Eigen::TransformationMatrix& frame, const std::string& parentFrame, const std::string& childFrame);

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
