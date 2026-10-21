#ifndef FER_SIM_ROBOT_VISUALIZER_HPP
#define FER_SIM_ROBOT_VISUALIZER_HPP

#include "fer_model/panda_model.hpp"
#include "fer_msgs/msg/state_data.hpp"
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace fer {

class RobotVisualizer : public rclcpp::Node {
public:
    RobotVisualizer();

private:
    void StateDataCB(const fer_msgs::msg::StateData& message);
    rclcpp::Subscription<fer_msgs::msg::StateData>::SharedPtr stateDataSub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPub_;
};

}

#endif
