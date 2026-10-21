#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include "rclcpp/rclcpp.hpp"
#include "fer_driver/latest_value.hpp"
#include "fer_msgs/conversions.hpp"
#include "fer_msgs/fer_defines.hpp"
#include "fer_msgs/msg/control_data.hpp"
#include "fer_msgs/msg/state_data.hpp"
#include "fer_msgs/srv/driver_command.hpp"

namespace fer {
class DriverNode : public rclcpp::Node {
  using Service = fer_msgs::srv::DriverCommand;
  struct Command {
    std::array<double, 7> velocity{};
    double received{0.};
    bool enabled{false};
  };
  struct Snapshot {
    std::array<double, 7> q{}, dq{}, tau{};
    std::array<double, 16> pose{{1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.}};
    std::array<double, 42> jacobian{};
    double success_rate{0.};
    bool connected{false}, active{false}, ready{false}, error{false};
    uint8_t mode{0};
  };
 public:
  DriverNode() : Node("driver_node") {
    address_ = declare_parameter("robot_ip", "172.16.0.2");
    timeout_ = declare_parameter("command_timeout", .20);
    max_velocity_ = declare_parameter("max_joint_velocity", .35);
    const double rate = declare_parameter("publish_rate", 100.);
    enforce_realtime_ = declare_parameter("enforce_realtime", true);
    base_frame_ = declare_parameter("base_frame", "panda_link0");
    if (!std::isfinite(timeout_) || timeout_ <= 0. || !std::isfinite(max_velocity_) ||
        max_velocity_ <= 0. || max_velocity_ > 2.175 || !std::isfinite(rate) || rate < 10. || rate > 1000.)
      throw std::invalid_argument("Invalid driver timeout, velocity or publish rate");
    state_pub_ = create_publisher<fer_msgs::msg::StateData>(topics::state_data, rclcpp::SensorDataQoS().keep_last(1));
    joints_pub_ = create_publisher<sensor_msgs::msg::JointState>(topics::joint_states, rclcpp::SensorDataQoS().keep_last(1));
    command_sub_ = create_subscription<fer_msgs::msg::ControlData>(topics::ctrl_data, rclcpp::QoS(1).reliable(),
      [this](const fer_msgs::msg::ControlData& msg) {
        Command command; command.received = monotonic(); command.enabled = msg.control_enabled;
        for (std::size_t i = 0; i < kJoints; ++i) {
          if (!std::isfinite(msg.joint_velocities[i]) || std::abs(msg.joint_velocities[i]) > max_velocity_ + 1e-9) {
            desired_enabled_.store(false); command.enabled = false; command.velocity.fill(0.);
            set_error("Invalid/out-of-range controller velocity");
            commands_.publish(command); return;
          }
          command.velocity[i] = msg.joint_velocities[i];
        }
        commands_.publish(command);
      });
    service_ = create_service<Service>(topics::driver_command,
      [this](Service::Request::SharedPtr request, Service::Response::SharedPtr response) { operation(*request, *response); });
    timer_ = create_wall_timer(std::chrono::duration<double>(1. / rate), [this] { publish(); });
    worker_ = std::thread([this] { work(); });
  }
  ~DriverNode() override {
    running_.store(false);
    desired_enabled_.store(false);
    if (worker_.joinable()) worker_.join();
  }
 private:
  static double monotonic() noexcept {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  void set_error(const std::string& message) {
    fault_.store(true); desired_enabled_.store(false);
    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
    std::lock_guard<std::mutex> lock(error_mutex_); error_ = message;
  }
  void operation(const Service::Request& req, Service::Response& res) {
    res.success = true;
    if (req.command_type == "disable") {
      desired_enabled_.store(false); res.message = "Disable requested";
    } else if (req.command_type == "enable") {
      if (fault_.load() || recovering_.load() || active_.load() || !ready_.load()) {
        res.success = false; res.message = "Driver not ready, already controlling, or recovery required";
      } else { desired_enabled_.store(true); res.message = "Enable accepted; awaiting fresh control data"; }
    } else if (req.command_type == "recover") {
      if (!connected_.load()) {
        res.success = false; res.message = "No robot connection; restart the driver";
      } else if (active_.load() || desired_enabled_.load() || recovering_.exchange(true)) {
        res.success = false; res.message = "Disable and wait for control to stop before recovery";
      } else { recover_.store(true); res.message = "Recovery queued"; }
    } else { res.success = false; res.message = "Unsupported arm driver command"; }
  }
  Snapshot snapshot(const franka::RobotState& state, const franka::Model& model, bool active) {
    Snapshot value;
    value.q = state.q; value.dq = state.dq; value.tau = state.tau_J;
    value.pose = state.O_T_EE;
    value.jacobian = model.zeroJacobian(franka::Frame::kEndEffector, state);
    value.connected = true; value.active = active;
    value.error = static_cast<bool>(state.current_errors) || fault_.load();
    value.ready = !value.error && (state.robot_mode == franka::RobotMode::kIdle || state.robot_mode == franka::RobotMode::kMove);
    value.mode = static_cast<uint8_t>(state.robot_mode);
    value.success_rate = state.control_command_success_rate;
    ready_.store(value.ready);
    connected_.store(true);
    return value;
  }
  void work() {
    Snapshot last;
    try {
      franka::Robot robot(address_, enforce_realtime_ ? franka::RealtimeConfig::kEnforce : franka::RealtimeConfig::kIgnore);
      auto model = robot.loadModel();
      Command command;
      while (running_.load()) {
        if (recover_.exchange(false)) {
          try {
            robot.automaticErrorRecovery();
            fault_.store(false);
            { std::lock_guard<std::mutex> lock(error_mutex_); error_.clear(); }
          } catch (const std::exception& e) { set_error(e.what()); }
          recovering_.store(false);
        }
        commands_.consume(command);
        if (desired_enabled_.load() && !fault_.load() && command.enabled && monotonic() - command.received <= timeout_) {
          bool stopping = false, watchdog = false;
          active_.store(true);
          try {
            robot.control([&](const franka::RobotState& state, franka::Duration period) -> franka::JointVelocities {
              commands_.consume(command);
              const bool expired = monotonic() - command.received > timeout_;
              if (!running_.load() || !desired_enabled_.load() || !command.enabled || expired || fault_.load()) {
                stopping = true; watchdog |= expired; desired_enabled_.store(false);
              }
              std::array<double, 7> target{};
              if (!stopping && period.toSec() > 0.) target = command.velocity;
              last = snapshot(state, model, true);
              snapshots_.publish(last);
              double speed = 0., acceleration = 0.;
              for (std::size_t i = 0; i < kJoints; ++i) {
                speed = std::max(speed, std::abs(state.dq_d[i]));
                acceleration = std::max(acceleration, std::abs(state.ddq_d[i]));
              }
              franka::JointVelocities output(target);
              if (stopping && speed < 1e-5 && acceleration < 1e-3) return franka::MotionFinished(output);
              return output;
            // libfranka applies its version-specific velocity/acceleration/jerk limiter
            // and the standard 100 Hz low-pass filter after this callback.
            }, franka::ControllerMode::kJointImpedance, true, 100.0);
            if (watchdog) set_error("Controller command watchdog expired; recover before enabling");
          } catch (const franka::Exception& e) { set_error(e.what()); }
          active_.store(false);
          desired_enabled_.store(false);
        } else {
          last = snapshot(robot.readOnce(), model, false);
          snapshots_.publish(last);
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
      }
    } catch (const std::exception& e) {
      set_error(e.what()); ready_.store(false); active_.store(false); connected_.store(false);
      last.connected = false; last.active = false; last.ready = false; last.error = true;
      snapshots_.publish(last);
    }
  }
  void publish() {
    Snapshot value;
    if (!snapshots_.consume(value)) return;  // Never refresh the timestamp of an old state.
    fer_msgs::msg::StateData msg;
    msg.header.stamp = now(); msg.header.frame_id = base_frame_;
    msg.connected = value.connected; msg.ready = value.ready;
    msg.control_active = value.active; msg.has_error = value.error;
    msg.robot_mode = value.mode; msg.control_command_success_rate = value.success_rate;
    { std::lock_guard<std::mutex> lock(error_mutex_); msg.error = error_; }
    msg.joint_state.header = msg.header; msg.joint_state.name = ros::joint_names();
    msg.joint_state.position.assign(value.q.begin(), value.q.end());
    msg.joint_state.velocity.assign(value.dq.begin(), value.dq.end());
    msg.joint_state.effort.assign(value.tau.begin(), value.tau.end());
    Pose pose = Pose::Identity(); pose.matrix() = Eigen::Map<const Eigen::Matrix4d>(value.pose.data());
    msg.tool_pose = ros::pose_message(pose); msg.zero_jacobian = value.jacobian;
    state_pub_->publish(msg); joints_pub_->publish(msg.joint_state);
  }
  std::string address_, base_frame_, error_;
  double timeout_, max_velocity_;
  bool enforce_realtime_;
  std::atomic<bool> running_{true}, desired_enabled_{false}, recover_{false},
      recovering_{false}, fault_{false}, ready_{false}, active_{false}, connected_{false};
  LatestValue<Command> commands_;
  LatestValue<Snapshot> snapshots_;
  std::mutex error_mutex_;
  std::thread worker_;
  rclcpp::Publisher<fer_msgs::msg::StateData>::SharedPtr state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_pub_;
  rclcpp::Subscription<fer_msgs::msg::ControlData>::SharedPtr command_sub_;
  rclcpp::Service<Service>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace fer

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<fer::DriverNode>()); }
  catch (const std::exception& e) { RCLCPP_FATAL(rclcpp::get_logger("fer_driver"), "%s", e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
