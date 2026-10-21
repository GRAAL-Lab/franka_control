#include "fer_tcp_bridge/franka_tcp_bridge.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<fer::FrankaTCPBridge>());
    } catch (const std::exception& exception) {
        RCLCPP_FATAL(rclcpp::get_logger("fer_tcp_bridge"), "%s", exception.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
