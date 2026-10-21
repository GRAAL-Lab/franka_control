#include "fer_ctrl/dynamic_control_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<fer::DynamicControlNode>());
    } catch (const std::exception& exception) {
        RCLCPP_FATAL(rclcpp::get_logger("fer_ctrl"), "%s", exception.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
