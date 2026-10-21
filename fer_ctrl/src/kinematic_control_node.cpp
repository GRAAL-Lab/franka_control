#include "fer_ctrl/kinematic_controller.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<fer::KinematicController>());
    } catch (const std::exception& exception) {
        RCLCPP_FATAL(rclcpp::get_logger("fer_ctrl"), "%s", exception.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
