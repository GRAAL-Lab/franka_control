#include "fer_driver/franka_gripper.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
        executor.add_node(std::make_shared<fer::FrankaGripper>());
        executor.spin();
    } catch (const std::exception& exception) {
        RCLCPP_FATAL(rclcpp::get_logger("fer_gripper"), "%s", exception.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
