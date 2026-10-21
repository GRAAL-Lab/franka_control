#include "fer_sim/robot_visualizer.hpp"
#include "fer_msgs/conversions.hpp"
#include "fer_msgs/fer_defines.hpp"

using std::placeholders::_1;

namespace fer {

RobotVisualizer::RobotVisualizer()
    : Node("visualizer_node")
{
    markerPub_ = create_publisher<visualization_msgs::msg::Marker>("visualization/arm", 1);
    stateDataSub_ = create_subscription<fer_msgs::msg::StateData>(topicnames::state_data,
        rclcpp::SensorDataQoS().keep_last(1), std::bind(&RobotVisualizer::StateDataCB, this, _1));
}

void RobotVisualizer::StateDataCB(const fer_msgs::msg::StateData& message)
{
    if (!message.connected || message.joint_state.position.size() != ARM_JOINTS) {
        return;
    }
    JointVector jointPosition = Eigen::Map<const JointVector>(message.joint_state.position.data());
    if (!jointPosition.allFinite()) {
        return;
    }

    KinematicData kinematics = PandaModel().ComputeKinematics(jointPosition);
    visualization_msgs::msg::Marker marker;
    marker.header = message.header;
    marker.ns = "fer_arm";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.045;
    marker.color.r = 0.8;
    marker.color.g = 0.85;
    marker.color.b = 0.9;
    marker.color.a = 1.0;
    marker.lifetime.nanosec = 500000000;
    for (const Pose& jointFrame : kinematics.jointFrames) {
        geometry_msgs::msg::Point point;
        point.x = jointFrame.translation().x();
        point.y = jointFrame.translation().y();
        point.z = jointFrame.translation().z();
        marker.points.push_back(point);
    }
    marker.points.push_back(message.tool_pose.position);
    markerPub_->publish(marker);
}

}
