#include "fer_sim/simulator.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<fer::Simulator>());
    } catch (const std::exception& exception) {
        RCLCPP_FATAL(rclcpp::get_logger("fer_sim"), "%s", exception.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
