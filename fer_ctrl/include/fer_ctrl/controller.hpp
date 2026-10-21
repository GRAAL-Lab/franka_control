#pragma once

#include <string>
#include "fer_ctrl/commands/command.hpp"
#include "fer_ctrl/states/state.hpp"

namespace fer {
struct Configuration {
  double joint_gain{1.5}, linear_gain{1.5}, angular_gain{1.5};
  double max_joint_velocity{0.35}, max_linear_velocity{0.10}, max_angular_velocity{0.3};
  double joint_tolerance{0.01}, linear_tolerance{0.002}, angular_tolerance{0.01};
  double joint_margin{0.05}, damping{0.03}, centering_gain{0.05};
  double feedback_timeout{0.2}, velocity_timeout{0.3}, move_timeout{30.}, enable_timeout{5.};
  Joints parked{(Joints() << 0., -.4, 0., -2.7, 0., 2.3, .7853981633974483).finished()};
  Joints unparked{kHome};
  void validate() const;
};

struct Feedback {
  Joints q{kHome}, dq{Joints::Zero()};
  Pose tool{Pose::Identity()};
  Jacobian jacobian{Jacobian::Zero()};
  bool connected{false}, ready{false}, active{false}, error{false};
};

struct Result { bool success; std::string message; };
struct Output {
  Joints velocity{Joints::Zero()};
  double joint_error{0.}, linear_error{0.}, angular_error{0.};
  bool joint_limit_active{false};
};

// ROS-independent command/state/task layer. All calls belong to one executor thread.
class Controller {
 public:
  explicit Controller(Configuration config = {});
  bool feedback(const Feedback& feedback, double time);
  Result execute(const commands::Command& command, double time);
  Output update(double time);
  void fail(const std::string& reason);
  states::State state() const { return state_; }
  bool enabled() const;
  const Pose& target() const { return pose_goal_; }
  const std::string& status() const { return status_; }
 private:
  bool fresh(double time) const;
  void transition(states::State state, double time, const std::string& status);
  Configuration config_;
  Feedback feedback_;
  bool received_{false};
  double feedback_time_{0.}, state_time_{0.}, velocity_time_{0.};
  states::State state_{states::State::Disabled};
  Joints joint_goal_{kHome}, velocity_goal_{Joints::Zero()};
  Pose pose_goal_{Pose::Identity()};
  std::string status_{"Waiting for feedback and enable command"};
};
}  // namespace fer
