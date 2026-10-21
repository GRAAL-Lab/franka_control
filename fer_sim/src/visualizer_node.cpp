#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "fer_msgs/conversions.hpp"
#include "fer_msgs/fer_defines.hpp"
#include "fer_msgs/msg/state_data.hpp"

namespace fer {
class VisualizerNode : public rclcpp::Node {
 public:
  VisualizerNode() : Node("visualizer_node") {
    pub_ = create_publisher<visualization_msgs::msg::Marker>("visualization/arm", 1);
    sub_ = create_subscription<fer_msgs::msg::StateData>(topics::state_data, rclcpp::SensorDataQoS().keep_last(1),
      [this](const fer_msgs::msg::StateData& msg) {
        if (!msg.connected || msg.joint_state.position.size() != 7) return;
        const Joints q = Eigen::Map<const Joints>(msg.joint_state.position.data());
        if (!q.allFinite()) return;
        const auto kin = PandaModel().evaluate(q);
        visualization_msgs::msg::Marker marker;
        marker.header = msg.header; marker.ns = "fer_arm"; marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.; marker.scale.x = .045;
        marker.color.r = .8; marker.color.g = .85; marker.color.b = .9; marker.color.a = 1.;
        marker.lifetime.nanosec = 500000000;
        for (const auto& link : kin.links) {
          geometry_msgs::msg::Point point;
          point.x = link.translation().x(); point.y = link.translation().y(); point.z = link.translation().z();
          marker.points.push_back(point);
        }
        marker.points.push_back(msg.tool_pose.position);
        pub_->publish(marker);
      });
  }
 private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_;
  rclcpp::Subscription<fer_msgs::msg::StateData>::SharedPtr sub_;
};
}  // namespace fer

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fer::VisualizerNode>());
  rclcpp::shutdown(); return 0;
}
