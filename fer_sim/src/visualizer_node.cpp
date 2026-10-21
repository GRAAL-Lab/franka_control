#include "fer_sim/robot_visualizer.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<fer::RobotVisualizer>());
    } catch (const std::exception& exception) {
        RCLCPP_FATAL(rclcpp::get_logger("fer_sim"), "%s", exception.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
