#ifndef FER_SIM_ROBOT_VISUALIZER_HPP
#define FER_SIM_ROBOT_VISUALIZER_HPP

#include "fer_model/panda_model.hpp"
#include "fer_msgs/msg/state_data.hpp"
#include "fer_msgs/msg/control_data.hpp"
#include "fer_msgs/srv/control_command.hpp"
#include <interactive_markers/interactive_marker_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <optional>

namespace fer {

class RobotVisualizer : public rclcpp::Node {
public:
    RobotVisualizer();

private:
    using CommandClient = rclcpp::Client<fer_msgs::srv::ControlCommand>;

    void LoadConfiguration();
    void Initialization();
    void StateDataCB(const fer_msgs::msg::StateData& message);
    void ControlDataCB(const fer_msgs::msg::ControlData& message);
    void Run();
    void PublishMarkers();
    void PublishToTF(const Pose& frame, const std::string& childFrame);
    void make6DofMarker(const geometry_msgs::msg::Pose& pose);
    void interactiveMarkerCB(
        const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);

    std::string baseFrame_;
    std::string jointFramePrefix_;
    std::string controlFrame_;
    double loopRate_ = 100.0;
    double lastFeedbackTime_ = 0.0;
    double commandSendTime_ = 0.0;
    bool receivingFeedback_ = false;
    bool dragging_ = false;
    bool sendCommandToController_ = false;
    KinematicData kinematics_;
    Pose toolFrame_ = Pose::Identity();
    Pose controlFramePose_ = Pose::Identity();
    fer_msgs::msg::ControlData controlDataMsg_;
    fer_msgs::srv::ControlCommand::Request commandRequest_;
    std::optional<CommandClient::FutureAndRequestId> pendingCommand_;
    std::vector<visualization_msgs::msg::Marker> planeMarkers_;
    std::unique_ptr<interactive_markers::InteractiveMarkerServer> intMarkerServer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfPublisher_;
    CommandClient::SharedPtr controlCommandClient_;
    rclcpp::TimerBase::SharedPtr runTimer_;
    rclcpp::Subscription<fer_msgs::msg::StateData>::SharedPtr stateDataSub_;
    rclcpp::Subscription<fer_msgs::msg::ControlData>::SharedPtr controlDataSub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualizationPub_;
};

}

#endif
