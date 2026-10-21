#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "fer_msgs/conversions.hpp"
#include "fer_msgs/fer_defines.hpp"
#include "fer_msgs/msg/control_data.hpp"
#include "fer_msgs/msg/state_data.hpp"
#include "fer_msgs/msg/gripper_state.hpp"
#include "fer_msgs/srv/driver_command.hpp"

namespace fer {
class SimulatorNode : public rclcpp::Node {
  using Service = fer_msgs::srv::DriverCommand;
 public:
  SimulatorNode() : Node("simulator_node"), model_(declare_parameter("tool_offset", .1034)) {
    const double rate = declare_parameter("loop_rate", 100.);
    timeout_ = declare_parameter("command_timeout", .20);
    acceleration_ = declare_parameter("max_joint_acceleration", .6);
    max_velocity_ = declare_parameter("max_joint_velocity", .35);
    base_frame_ = declare_parameter("base_frame", "panda_link0");
    const auto initial = declare_parameter("initial_joints", std::vector<double>(kHome.data(), kHome.data() + 7));
    if (initial.size() != 7) throw std::invalid_argument("initial_joints must have seven values");
    q_ = Eigen::Map<const Joints>(initial.data());
    if (!within_limits(q_) || !std::isfinite(rate) || rate < 10. || rate > 1000. ||
        !std::isfinite(timeout_) || timeout_ <= 0. || !std::isfinite(acceleration_) || acceleration_ <= 0. ||
        !std::isfinite(max_velocity_) || max_velocity_ <= 0. || max_velocity_ > 2.175)
      throw std::invalid_argument("Invalid simulator parameters");
    state_pub_ = create_publisher<fer_msgs::msg::StateData>(topics::state_data, rclcpp::SensorDataQoS().keep_last(1));
    joints_pub_ = create_publisher<sensor_msgs::msg::JointState>(topics::joint_states, rclcpp::SensorDataQoS().keep_last(1));
    grip_pub_ = create_publisher<fer_msgs::msg::GripperState>(topics::gripper_state, 1);
    control_sub_ = create_subscription<fer_msgs::msg::ControlData>(topics::ctrl_data, rclcpp::QoS(1).reliable(),
      [this](const fer_msgs::msg::ControlData& msg) {
        command_ = Eigen::Map<const Joints>(msg.joint_velocities.data());
        command_enabled_ = msg.control_enabled;
        received_ = monotonic();
        if (!command_.allFinite() || command_.cwiseAbs().maxCoeff() > max_velocity_ + 1e-9) {
          command_.setZero(); active_ = requested_ = false; fault_ = true;
          error_ = "Invalid/out-of-range controller velocity";
        }
      });
    arm_service_ = create_service<Service>(topics::driver_command,
      [this](Service::Request::SharedPtr req, Service::Response::SharedPtr res) {
        res->success = true;
        if (req->command_type == "enable" && !fault_ && !active_) requested_ = true;
        else if (req->command_type == "disable") requested_ = false;
        else if (req->command_type == "recover" && !active_) { requested_ = false; fault_ = false; error_.clear(); }
        else { res->success = false; res->message = "Unsupported command or simulator not ready"; return; }
        res->message = "Simulator operation accepted";
      });
    grip_service_ = create_service<Service>(topics::gripper_command,
      [this](Service::Request::SharedPtr req, Service::Response::SharedPtr res) {
        res->success = true;
        if (req->command_type == "stop") { grip_target_ = width_; grasped_ = false; }
        else if (req->command_type == "home") { grip_target_ = .08; grip_speed_ = .05; grasped_ = false; }
        else if ((req->command_type == "move" || req->command_type == "grasp") &&
            std::isfinite(req->width) && req->width >= 0. && req->width <= .08 &&
            std::isfinite(req->speed) && req->speed > 0. && req->speed <= .2 &&
            std::isfinite(req->force) && req->force >= 5. && req->force <= 70.) {
          grip_target_ = req->width; grip_speed_ = req->speed;
          // There are no simulated objects/contact forces; never report a successful grasp.
          grasped_ = false;
        } else { res->success = false; res->message = "Invalid gripper command"; return; }
        res->message = "Simulated gripper command accepted";
      });
    previous_ = monotonic();
    timer_ = create_wall_timer(std::chrono::duration<double>(1. / rate), [this] { run(); });
  }
 private:
  static double monotonic() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  void run() {
    const double time = monotonic();
    const double dt = std::clamp(time - previous_, 0., .05);
    previous_ = time;
    const bool fresh = time - received_ <= timeout_;
    if (requested_ && !active_ && fresh && command_enabled_ && !fault_) active_ = true;
    if (active_ && !fresh) { requested_ = false; fault_ = true; error_ = "Controller command watchdog expired"; }
    Joints target = Joints::Zero();
    if (active_ && requested_ && command_enabled_ && fresh && !fault_) target = command_;
    else if (active_) requested_ = false;
    for (int i = 0; i < 7; ++i) {
      dq_[i] += std::clamp(target[i] - dq_[i], -acceleration_ * dt, acceleration_ * dt);
      q_[i] = std::clamp(q_[i] + dq_[i] * dt, kLower[i], kUpper[i]);
      if ((q_[i] == kLower[i] && dq_[i] < 0.) || (q_[i] == kUpper[i] && dq_[i] > 0.)) dq_[i] = 0.;
    }
    if (active_ && !requested_ && dq_.cwiseAbs().maxCoeff() < 1e-6) active_ = false;
    const auto kin = model_.evaluate(q_);
    fer_msgs::msg::StateData msg;
    msg.header.stamp = now(); msg.header.frame_id = base_frame_;
    msg.connected = true; msg.ready = !fault_; msg.control_active = active_;
    msg.has_error = fault_; msg.error = error_; msg.robot_mode = active_ ? 2 : 1;
    msg.control_command_success_rate = 1.;
    msg.joint_state.header = msg.header; msg.joint_state.name = ros::joint_names();
    msg.joint_state.position.assign(q_.data(), q_.data() + 7);
    msg.joint_state.velocity.assign(dq_.data(), dq_.data() + 7);
    msg.joint_state.effort.assign(7, 0.);
    msg.tool_pose = ros::pose_message(kin.tool);
    std::copy_n(kin.jacobian.data(), 42, msg.zero_jacobian.begin());
    state_pub_->publish(msg); joints_pub_->publish(msg.joint_state);
    width_ += std::clamp(grip_target_ - width_, -grip_speed_ * dt, grip_speed_ * dt);
    fer_msgs::msg::GripperState grip;
    grip.header = msg.header; grip.width = width_; grip.max_width = .08;
    grip.is_grasped = grasped_; grip.busy = std::abs(grip_target_ - width_) > 1e-5;
    grip_pub_->publish(grip);
  }
  PandaModel model_;
  Joints q_{kHome}, dq_{Joints::Zero()}, command_{Joints::Zero()};
  double timeout_, acceleration_, max_velocity_, previous_{0.}, received_{0.};
  double width_{.08}, grip_target_{.08}, grip_speed_{.05};
  bool active_{false}, requested_{false}, command_enabled_{false}, fault_{false}, grasped_{false};
  std::string base_frame_, error_;
  rclcpp::Subscription<fer_msgs::msg::ControlData>::SharedPtr control_sub_;
  rclcpp::Publisher<fer_msgs::msg::StateData>::SharedPtr state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_pub_;
  rclcpp::Publisher<fer_msgs::msg::GripperState>::SharedPtr grip_pub_;
  rclcpp::Service<Service>::SharedPtr arm_service_, grip_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace fer

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<fer::SimulatorNode>()); }
  catch (const std::exception& e) { RCLCPP_FATAL(rclcpp::get_logger("fer_sim"), "%s", e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
