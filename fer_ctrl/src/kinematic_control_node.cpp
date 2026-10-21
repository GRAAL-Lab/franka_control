#include <chrono>
#include <future>
#include <memory>
#include <stdexcept>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "fer_ctrl/controller.hpp"
#include "fer_msgs/conversions.hpp"
#include "fer_msgs/fer_defines.hpp"
#include "fer_msgs/msg/control_data.hpp"
#include "fer_msgs/msg/state_data.hpp"
#include "fer_msgs/msg/task_status.hpp"
#include "fer_msgs/srv/control_command.hpp"
#include "fer_msgs/srv/driver_command.hpp"

namespace fer {
class KinematicControlNode : public rclcpp::Node {
  using Driver = fer_msgs::srv::DriverCommand;
  using Service = fer_msgs::srv::ControlCommand;
  using Client = rclcpp::Client<Driver>;
 public:
  KinematicControlNode() : Node("kinematic_control_node"), controller_(configuration()) {
    const double rate = declare_parameter("loop_rate", 100.);
    if (!std::isfinite(rate) || rate < 10. || rate > 1000.) throw std::invalid_argument("loop_rate must be in 10..1000 Hz");
    base_frame_ = declare_parameter("base_frame", "panda_link0");
    tool_frame_ = declare_parameter("tool_frame", "fer_tool");
    control_pub_ = create_publisher<fer_msgs::msg::ControlData>(topics::ctrl_data, rclcpp::QoS(1).reliable());
    task_pub_ = create_publisher<fer_msgs::msg::TaskStatus>(topics::task_status, 1);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    driver_ = create_client<Driver>(topics::driver_command);
    gripper_ = create_client<Driver>(topics::gripper_command);
    state_sub_ = create_subscription<fer_msgs::msg::StateData>(topics::state_data, rclcpp::SensorDataQoS().keep_last(1),
      [this](const fer_msgs::msg::StateData& msg) { receive(msg); });
    service_ = create_service<Service>(topics::control_command,
      [this](Service::Request::SharedPtr request, Service::Response::SharedPtr response) { command(*request, *response); });
    timer_ = create_wall_timer(std::chrono::duration<double>(1. / rate), [this] { run(); });
    RCLCPP_INFO(get_logger(), "FER controller ready; waiting for feedback and enable");
  }
 private:
  struct Pending {
    Client::SharedPtr client;
    Client::FutureAndRequestId future;
    std::string operation;
    double time;
  };
  static double monotonic() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  Configuration configuration() {
    Configuration c;
    c.joint_gain = declare_parameter("joint_gain", c.joint_gain);
    c.linear_gain = declare_parameter("linear_gain", c.linear_gain);
    c.angular_gain = declare_parameter("angular_gain", c.angular_gain);
    c.max_joint_velocity = declare_parameter("max_joint_velocity", c.max_joint_velocity);
    c.max_linear_velocity = declare_parameter("max_linear_velocity", c.max_linear_velocity);
    c.max_angular_velocity = declare_parameter("max_angular_velocity", c.max_angular_velocity);
    c.joint_tolerance = declare_parameter("joint_tolerance", c.joint_tolerance);
    c.linear_tolerance = declare_parameter("linear_tolerance", c.linear_tolerance);
    c.angular_tolerance = declare_parameter("angular_tolerance", c.angular_tolerance);
    c.joint_margin = declare_parameter("joint_margin", c.joint_margin);
    c.damping = declare_parameter("damping", c.damping);
    c.centering_gain = declare_parameter("centering_gain", c.centering_gain);
    c.feedback_timeout = declare_parameter("feedback_timeout", c.feedback_timeout);
    c.velocity_timeout = declare_parameter("velocity_timeout", c.velocity_timeout);
    c.move_timeout = declare_parameter("move_timeout", c.move_timeout);
    c.enable_timeout = declare_parameter("enable_timeout", c.enable_timeout);
    auto posture = [this](const char* name, const Joints& initial) -> Joints {
      const auto values = declare_parameter(name, std::vector<double>(initial.data(), initial.data() + 7));
      if (values.size() != 7) throw std::invalid_argument(std::string(name) + " must have seven values");
      return Eigen::Map<const Joints>(values.data());
    };
    c.parked = posture("parked_joints", c.parked);
    c.unparked = posture("unparked_joints", c.unparked);
    return c;
  }
  void receive(const fer_msgs::msg::StateData& msg) {
    try {
      if (msg.joint_state.position.size() != 7 || msg.joint_state.velocity.size() != 7 ||
          msg.joint_state.name != ros::joint_names()) throw std::invalid_argument("Feedback must contain Panda joints 1..7 in order");
      Feedback f;
      f.q = Eigen::Map<const Joints>(msg.joint_state.position.data());
      f.dq = Eigen::Map<const Joints>(msg.joint_state.velocity.data());
      f.tool = ros::pose(msg.tool_pose);
      f.jacobian = Eigen::Map<const Jacobian>(msg.zero_jacobian.data());
      f.connected = msg.connected; f.ready = msg.ready;
      f.active = msg.control_active; f.error = msg.has_error;
      if (controller_.feedback(f, monotonic()) && f.connected) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header = msg.header; transform.header.frame_id = base_frame_;
        transform.child_frame_id = tool_frame_;
        transform.transform.translation.x = msg.tool_pose.position.x;
        transform.transform.translation.y = msg.tool_pose.position.y;
        transform.transform.translation.z = msg.tool_pose.position.z;
        transform.transform.rotation = msg.tool_pose.orientation;
        tf_->sendTransform(transform);
      }
    } catch (const std::exception& e) { controller_.fail(e.what()); }
  }
  void command(const Service::Request& req, Service::Response& res) {
    commands::Command cmd;
    cmd.type = commands::parse(req.command_type);
    cmd.relative = req.move_type == "relative";
    cmd.valid_move_type = req.move_type == "absolute" || cmd.relative;
    cmd.joints = req.joint_setpoint; cmd.joint_index = req.joint_index;
    cmd.xyz = Eigen::Map<const Eigen::Vector3d>(req.target_position.data());
    cmd.rpy = Eigen::Map<const Eigen::Vector3d>(req.target_orientation.data());
    cmd.frame = req.frame_type;
    const bool grip = cmd.type == commands::Type::GripperMove || cmd.type == commands::Type::GripperGrasp ||
                      cmd.type == commands::Type::GripperHome || cmd.type == commands::Type::GripperStop;
    const bool arm = cmd.type == commands::Type::Enable || cmd.type == commands::Type::Disable || cmd.type == commands::Type::Recover;
    const auto client = grip ? gripper_ : driver_;
    if ((grip || arm) && !client->service_is_ready() && cmd.type != commands::Type::Disable) {
      res.success = false; res.res = "Requested driver service is unavailable"; return;
    }
    if (grip && (cmd.type == commands::Type::GripperMove || cmd.type == commands::Type::GripperGrasp) &&
        (!std::isfinite(req.gripper_setpoint) || req.gripper_setpoint < 0. || req.gripper_setpoint > .08 ||
         !std::isfinite(req.gripper_speed) || req.gripper_speed <= 0. || req.gripper_speed > .2 ||
         !std::isfinite(req.grasp_force) || req.grasp_force < 5. || req.grasp_force > 70.)) {
      res.success = false; res.res = "Use gripper width 0..0.08 m, speed (0,0.2] m/s, force 5..70 N"; return;
    }
    const auto result = controller_.execute(cmd, monotonic());
    res.success = result.success; res.res = result.message;
    if (!result.success) return;
    // Publish disable/enable intent before dispatching the corresponding driver operation.
    publish(controller_.update(monotonic()));
    if ((arm || grip) && client->service_is_ready()) {
      auto request = std::make_shared<Driver::Request>();
      request->command_type = grip ? req.command_type.substr(8) : req.command_type;
      request->width = req.gripper_setpoint; request->speed = req.gripper_speed;
      request->force = req.grasp_force;
      pending_.push_back({client, client->async_send_request(request), request->command_type, monotonic()});
      res.res += "; driver request queued";
    }
  }
  void run() {
    const double time = monotonic();
    for (auto it = pending_.begin(); it != pending_.end();) {
      bool erase = false;
      if (it->future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        const auto response = it->future.get();
        if (!response->success) {
          RCLCPP_ERROR(get_logger(), "%s: %s", it->operation.c_str(), response->message.c_str());
          if (it->client == driver_ && it->operation == "enable" && controller_.state() == states::State::Enabling)
            controller_.fail(response->message);
        }
        erase = true;
      } else if (time - it->time > 30.) {
        it->client->remove_pending_request(it->future);
        RCLCPP_ERROR(get_logger(), "%s: service timeout", it->operation.c_str());
        if (it->client == driver_ && controller_.state() == states::State::Enabling) controller_.fail("Driver service timeout");
        erase = true;
      }
      if (erase) it = pending_.erase(it); else ++it;
    }
    publish(controller_.update(time));
  }
  void publish(const Output& output) {
    fer_msgs::msg::ControlData msg;
    msg.header.stamp = now(); msg.header.frame_id = base_frame_;
    msg.ctrl_state = states::name(controller_.state());
    msg.control_enabled = controller_.enabled(); msg.status = controller_.status();
    std::copy_n(output.velocity.data(), 7, msg.joint_velocities.begin());
    msg.target_pose = ros::pose_message(controller_.target());
    control_pub_->publish(msg);
    fer_msgs::msg::TaskStatus task;
    task.header = msg.header; task.action = msg.ctrl_state;
    task.joint_error = output.joint_error; task.linear_error = output.linear_error;
    task.angular_error = output.angular_error; task.joint_limit_active = output.joint_limit_active;
    task_pub_->publish(task);
  }
  Controller controller_;
  std::string base_frame_, tool_frame_;
  Client::SharedPtr driver_, gripper_;
  std::vector<Pending> pending_;
  rclcpp::Publisher<fer_msgs::msg::ControlData>::SharedPtr control_pub_;
  rclcpp::Publisher<fer_msgs::msg::TaskStatus>::SharedPtr task_pub_;
  rclcpp::Subscription<fer_msgs::msg::StateData>::SharedPtr state_sub_;
  rclcpp::Service<Service>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
};
}  // namespace fer

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<fer::KinematicControlNode>()); }
  catch (const std::exception& e) { RCLCPP_FATAL(rclcpp::get_logger("fer_ctrl"), "%s", e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown();
  return 0;
}
