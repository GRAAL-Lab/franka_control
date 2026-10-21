#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <franka/gripper.h>
#include "rclcpp/rclcpp.hpp"
#include "fer_msgs/fer_defines.hpp"
#include "fer_msgs/msg/gripper_state.hpp"
#include "fer_msgs/srv/driver_command.hpp"

namespace fer {
class GripperNode : public rclcpp::Node {
  using Service = fer_msgs::srv::DriverCommand;
 public:
  GripperNode() : Node("gripper_node") {
    gripper_ = std::make_unique<franka::Gripper>(declare_parameter("robot_ip", "172.16.0.2"));
    group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    pub_ = create_publisher<fer_msgs::msg::GripperState>(topics::gripper_state, 1);
    service_ = create_service<Service>(topics::gripper_command,
      [this](Service::Request::SharedPtr req, Service::Response::SharedPtr res) { command(*req, *res); },
      rclcpp::ServicesQoS(), group_);
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { publish(); }, group_);
  }
 private:
  void command(const Service::Request& req, Service::Response& res) {
    if (req.command_type == "stop") {
      try { res.success = gripper_->stop(); res.message = res.success ? "Gripper stopped" : "Gripper stop failed"; }
      catch (const std::exception& e) { res.success = false; res.message = e.what(); }
      return;
    }
    if (req.command_type != "home" && req.command_type != "move" && req.command_type != "grasp") {
      res.success = false; res.message = "Unsupported gripper command"; return;
    }
    if (req.command_type != "home" && (!std::isfinite(req.width) || req.width < 0. || req.width > .08 ||
        !std::isfinite(req.speed) || req.speed <= 0. || req.speed > .2 ||
        !std::isfinite(req.force) || req.force < 5. || req.force > 70.)) {
      res.success = false; res.message = "Invalid width, speed or force"; return;
    }
    if (busy_.exchange(true)) { res.success = false; res.message = "Gripper busy"; return; }
    try {
      if (req.command_type == "home") res.success = gripper_->homing();
      else if (req.command_type == "move") res.success = gripper_->move(req.width, req.speed);
      else res.success = gripper_->grasp(req.width, req.speed, req.force);
      res.message = res.success ? "Gripper command completed" : "Gripper command failed";
    } catch (const std::exception& e) { res.success = false; res.message = e.what(); }
    { std::lock_guard<std::mutex> lock(mutex_); error_ = res.success ? "" : res.message; }
    busy_.store(false);
  }
  void publish() {
    // Do not queue a read behind a long move; another executor thread remains available for stop.
    if (busy_.load()) {
      std::lock_guard<std::mutex> lock(mutex_);
      auto msg = last_state_;
      msg.busy = true; msg.error = error_;
      pub_->publish(msg);
      return;
    }
    try {
      const auto state = gripper_->readOnce();
      fer_msgs::msg::GripperState msg;
      msg.header.stamp = now(); msg.width = state.width; msg.max_width = state.max_width;
      msg.is_grasped = state.is_grasped; msg.busy = busy_.load();
      { std::lock_guard<std::mutex> lock(mutex_); msg.error = error_; last_state_ = msg; }
      pub_->publish(msg);
    } catch (const std::exception& e) {
      std::lock_guard<std::mutex> lock(mutex_); error_ = e.what();
    }
  }
  std::unique_ptr<franka::Gripper> gripper_;
  std::atomic<bool> busy_{false};
  std::mutex mutex_;
  std::string error_;
  fer_msgs::msg::GripperState last_state_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Publisher<fer_msgs::msg::GripperState>::SharedPtr pub_;
  rclcpp::Service<Service>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace fer

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(std::make_shared<fer::GripperNode>());
    executor.spin();
  } catch (const std::exception& e) { RCLCPP_FATAL(rclcpp::get_logger("fer_gripper"), "%s", e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
