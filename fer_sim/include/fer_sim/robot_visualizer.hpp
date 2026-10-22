/** @file robot_visualizer.hpp */

#ifndef FER_SIM_ROBOT_VISUALIZER_HPP
#define FER_SIM_ROBOT_VISUALIZER_HPP

#include "fer_model/franka_arm_model.hpp"
#include "fer_msgs/msg/state_data.hpp"
#include "fer_msgs/msg/control_data.hpp"
#include "fer_msgs/srv/control_command.hpp"
#include <interactive_markers/interactive_marker_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <optional>

namespace fer {

/**
 * @brief ROS-clock visualization of measured arm kinematics, obstacle planes and an interactive tool
 * goal.
 */
class RobotVisualizer : public rclcpp::Node {
public:
    /** @brief Load display configuration and create marker, feedback, command and TF interfaces. */
    RobotVisualizer();

private:
    using CommandClient = rclcpp::Client<fer_msgs::srv::ControlCommand>;

    /** @brief Read frame/rate parameters and build obstacle plane markers from kcl_fer.conf. */
    void LoadConfiguration();
    /** @brief Read obstacle planes from the controller configuration and build their base-frame markers. */
    void LoadObstacleMarkers(const std::string& configurationFile);
    /**
     * @brief Create ROS publishers, subscriptions, the interactive marker server and the node-clock
     * timer.
     */
    void InitializeROSInterfaces();
    /**
     * @brief Validate measured joints/tool pose, update the RML arm and initialize the tool control
     * marker.
     */
    void HandleStateData(const fer_msgs::msg::StateData& message);
    /** @brief Cache the controller state and current goal for interactive marker synchronization. */
    void HandleControlData(const fer_msgs::msg::ControlData& message);
    /**
     * @brief Process tool commands and render fresh feedback; publish interactive poses only when changed.
     * @details Apply pending drag feedback even when no programmatic pose update is needed.
     */
    void Run();
    /**
     * @brief Collect or expire the outstanding tool command and send a queued goal when feedback is
     * fresh.
     */
    void ProcessToolCommand(bool feedbackFresh);
    /** @brief Publish base-frame geometry and the serial TF chain from the base through joints 1..7. */
    void PublishMarkers();
    /**
     * @brief Broadcast a child-to-parent RML transform using the current node timestamp.
     * @param frame Child frame pose expressed in parentFrame.
     * @param parentFrame Parent ROS TF frame identifier.
     * @param childFrame Child ROS TF frame identifier.
     */
    void PublishToTF(const Eigen::TransformationMatrix& frame, const std::string& parentFrame,
        const std::string& childFrame);
    /** @brief Create the interactive translation/rotation controls at the given base-frame tool pose. */
    void CreateToolControlMarker(const geometry_msgs::msg::Pose& pose);
    /**
     * @brief Track drag feedback and queue an absolute base-frame Cartesian command when the mouse is
     * released.
     */
    void HandleToolControlFeedback(
        const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);

    std::string baseFrame_;
    std::string jointFramePrefix_;
    std::string controlFrame_;
    double loopRate_ = 100.0;
    double lastFeedbackTime_ = 0.0;
    double commandSendTime_ = 0.0;
    bool receivingFeedback_ = false;
    bool dragging_ = false;
    bool commandQueued_ = false;
    FrankaArmModel model_;
    Eigen::TransformationMatrix toolFrame_;
    Eigen::TransformationMatrix controlFramePose_;
    fer_msgs::msg::ControlData controlData_;
    fer_msgs::srv::ControlCommand::Request commandRequest_;
    std::optional<CommandClient::FutureAndRequestId> pendingCommand_;
    std::vector<visualization_msgs::msg::Marker> planeMarkers_;
    std::unique_ptr<interactive_markers::InteractiveMarkerServer> interactiveMarkerServer_;
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
