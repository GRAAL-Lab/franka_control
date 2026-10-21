#include "fer_ctrl/controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <Eigen/Cholesky>

namespace fer {
using states::State;
using commands::Type;

void Configuration::validate() const {
  for (double value : {joint_gain, linear_gain, angular_gain, max_joint_velocity,
       max_linear_velocity, max_angular_velocity, joint_tolerance, linear_tolerance,
       angular_tolerance, joint_margin, damping, feedback_timeout, velocity_timeout,
       move_timeout, enable_timeout}) {
    if (!std::isfinite(value) || value <= 0.) throw std::invalid_argument("Controller parameters must be finite and positive");
  }
  if (!std::isfinite(centering_gain) || centering_gain < 0. || max_joint_velocity > 2.175 ||
      joint_margin >= 0.5 * (kUpper - kLower).minCoeff() ||
      !within_limits(parked, joint_margin) || !within_limits(unparked, joint_margin)) {
    throw std::invalid_argument("Invalid centering gain, velocity limit, joint margin or park/unpark configuration");
  }
}

Controller::Controller(Configuration config) : config_(std::move(config)) { config_.validate(); }

bool Controller::feedback(const Feedback& value, double time) {
  const auto& rotation = value.tool.linear();
  if (!std::isfinite(time) || !value.q.allFinite() || !value.dq.allFinite() ||
      !value.tool.matrix().allFinite() || !value.jacobian.allFinite() ||
      (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() > 1e-5 ||
      std::abs(rotation.determinant() - 1.) > 1e-5) {
    fail("Invalid robot feedback");
    received_ = false;
    return false;
  }
  feedback_ = value;
  feedback_time_ = time;
  received_ = true;
  return true;
}

bool Controller::fresh(double time) const {
  return received_ && time >= feedback_time_ && time - feedback_time_ <= config_.feedback_timeout;
}
bool Controller::enabled() const { return state_ != State::Disabled && state_ != State::Fault; }
void Controller::transition(State state, double time, const std::string& status) {
  state_ = state;
  state_time_ = time;
  status_ = status;
  velocity_goal_.setZero();
}
void Controller::fail(const std::string& reason) {
  state_ = State::Fault;
  velocity_goal_.setZero();
  status_ = reason;
}

Result Controller::execute(const commands::Command& cmd, double time) {
  if (cmd.type == Type::Disable || cmd.type == Type::Recover) {
    transition(State::Disabled, time, cmd.type == Type::Recover ? "Recovery requested; enable again after completion" : "Disabled");
    return {true, status_};
  }
  if (cmd.type == Type::Stop) {
    if (state_ == State::Enabling) transition(State::Disabled, time, "Enable cancelled");
    else if (enabled() && state_ != State::Parked) transition(State::Idle, time, "Stopped");
    else velocity_goal_.setZero();
    return {true, "Stop accepted"};
  }
  if (cmd.type == Type::Unknown) return {false, "Unsupported command"};
  if (cmd.type == Type::GripperStop) return {true, "Gripper stop accepted"};
  if (!fresh(time) || !feedback_.connected) return {false, "No fresh robot feedback"};
  if (cmd.type == Type::Enable) {
    if (state_ == State::Fault || feedback_.error || !feedback_.ready) return {false, "Robot is not ready; resolve the error and recover first"};
    if (enabled()) return {false, "Controller is already enabled"};
    transition(State::Enabling, time, "Waiting for driver control loop");
    return {true, status_};
  }
  if (cmd.type == Type::GripperMove || cmd.type == Type::GripperGrasp ||
      cmd.type == Type::GripperHome || cmd.type == Type::GripperStop) return {true, "Gripper command accepted"};
  if (!enabled() || state_ == State::Enabling || !feedback_.active || feedback_.error) return {false, "Enable the controller before moving"};
  if (state_ == State::Parked && cmd.type != Type::Unpark) return {false, "Unpark before moving"};
  if (cmd.type == Type::Park || cmd.type == Type::Unpark) {
    joint_goal_ = cmd.type == Type::Park ? config_.parked : config_.unparked;
    transition(cmd.type == Type::Park ? State::Parking : State::Unparking, time, "Moving to configured posture");
    return {true, status_};
  }
  const bool position = cmd.type == Type::JointPosition || cmd.type == Type::SingleJointPosition;
  const bool velocity = cmd.type == Type::JointVelocity || cmd.type == Type::SingleJointVelocity;
  if (position || velocity) {
    const bool single = cmd.type == Type::SingleJointPosition || cmd.type == Type::SingleJointVelocity;
    if (cmd.joints.size() != (single ? 1u : kJoints) ||
        (single && (cmd.joint_index < 0 || cmd.joint_index >= 7))) return {false, "Expected seven joint setpoints, or one setpoint and an index in 0..6"};
    Joints goal = position ? feedback_.q : Joints::Zero();
    if (position && !cmd.valid_move_type) return {false, "move_type must be absolute or relative"};
    for (std::size_t i = 0; i < cmd.joints.size(); ++i) {
      const int joint = single ? cmd.joint_index : static_cast<int>(i);
      if (!std::isfinite(cmd.joints[i])) return {false, "Joint setpoints must be finite"};
      goal[joint] = cmd.joints[i] + (position && cmd.relative ? feedback_.q[joint] : 0.);
    }
    if (position && !within_limits(goal, config_.joint_margin)) return {false, "Joint target exceeds Panda limits and configured margin"};
    if (velocity && goal.cwiseAbs().maxCoeff() > config_.max_joint_velocity) return {false, "Joint velocity exceeds configured limit"};
    transition(position ? State::JointPosition : State::JointVelocity, time, "Joint command accepted");
    if (position) joint_goal_ = goal;
    else { velocity_goal_ = goal; velocity_time_ = time; }
    return {true, status_};
  }
  if (cmd.type == Type::Cartesian) {
    if (!cmd.valid_move_type || !cmd.xyz.allFinite() || !cmd.rpy.allFinite() ||
        (cmd.frame != 0 && cmd.frame != 1) || (!cmd.relative && cmd.frame != 1)) return {false, "Use finite xyz/rpy, absolute BASE, or relative BASE/EE"};
    const Pose requested = from_xyz_rpy(cmd.xyz, cmd.rpy);
    if (!cmd.relative) pose_goal_ = requested;
    else if (cmd.frame == 0) pose_goal_ = feedback_.tool * requested;
    else {
      pose_goal_ = feedback_.tool;
      pose_goal_.translation() += requested.translation();
      pose_goal_.linear() = requested.linear() * feedback_.tool.linear();
    }
    transition(State::Cartesian, time, "Cartesian command accepted");
    return {true, status_};
  }
  return {false, "Unsupported command"};
}

Output Controller::update(double time) {
  Output output;
  if (!fresh(time) || !feedback_.connected) {
    if (enabled()) transition(State::Disabled, time, "Feedback timeout/disconnection; enable again after reconnecting");
    return output;
  }
  if (feedback_.error && enabled()) { fail("Driver reported an error; recover explicitly"); return output; }
  if (state_ == State::Enabling) {
    if (feedback_.active) transition(State::Idle, time, "Enabled");
    else if (time - state_time_ > config_.enable_timeout) fail("Driver enable timed out");
    return output;
  }
  if (enabled() && !feedback_.active) { transition(State::Disabled, time, "Driver stopped; enable again to resume"); return output; }
  if (!enabled() || state_ == State::Idle || state_ == State::Parked) return output;
  if (state_ == State::JointVelocity) {
    if (time - velocity_time_ > config_.velocity_timeout) {
      transition(State::Idle, time, "Velocity command expired"); return output;
    }
    output.velocity = velocity_goal_;
  } else if (time - state_time_ > config_.move_timeout) {
    transition(State::Idle, time, "Move timed out"); return output;
  } else if (state_ == State::Cartesian) {
    const Twist error = pose_error(feedback_.tool, pose_goal_);
    output.linear_error = error.head<3>().norm();
    output.angular_error = error.tail<3>().norm();
    if (output.linear_error < config_.linear_tolerance && output.angular_error < config_.angular_tolerance) {
      transition(State::Idle, time, "Cartesian goal reached"); return output;
    }
    Twist reference = error;
    reference.head<3>() *= config_.linear_gain;
    reference.tail<3>() *= config_.angular_gain;
    reference.head<3>() *= std::min(1., config_.max_linear_velocity / std::max(reference.head<3>().norm(), 1e-12));
    reference.tail<3>() *= std::min(1., config_.max_angular_velocity / std::max(reference.tail<3>().norm(), 1e-12));
    const auto& j = feedback_.jacobian;
    const Eigen::Matrix<double, 6, 6> regularized = j * j.transpose() +
        config_.damping * config_.damping * Eigen::Matrix<double, 6, 6>::Identity();
    const Eigen::Matrix<double, 7, 6> inverse = j.transpose() * regularized.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());
    output.velocity = inverse * reference + (Eigen::Matrix<double, 7, 7>::Identity() - inverse * j) *
        (config_.centering_gain * (0.5 * (kLower + kUpper) - feedback_.q));
  } else {
    const Joints error = joint_goal_ - feedback_.q;
    output.joint_error = error.cwiseAbs().maxCoeff();
    if (output.joint_error < config_.joint_tolerance && feedback_.dq.cwiseAbs().maxCoeff() < .03) {
      transition(state_ == State::Parking ? State::Parked : State::Idle, time, "Joint goal reached"); return output;
    }
    output.velocity = config_.joint_gain * error;
  }
  // Velocity bounds and joint-limit dampers constrain every motion task.
  const double scale = std::max(1., output.velocity.cwiseAbs().maxCoeff() / config_.max_joint_velocity);
  output.velocity /= scale;
  for (int i = 0; i < 7; ++i) {
    const double lower = std::min(0., -config_.joint_gain * (feedback_.q[i] - kLower[i] - config_.joint_margin));
    const double upper = std::max(0., config_.joint_gain * (kUpper[i] - config_.joint_margin - feedback_.q[i]));
    const double bounded = std::clamp(output.velocity[i], lower, upper);
    output.joint_limit_active |= bounded != output.velocity[i];
    output.velocity[i] = bounded;
  }
  if (!output.velocity.allFinite()) { fail("Nonfinite controller output"); output.velocity.setZero(); }
  return output;
}
}  // namespace fer
