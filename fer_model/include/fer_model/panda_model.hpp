#pragma once

#include <array>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace fer {
inline constexpr std::size_t kJoints = 7;
inline constexpr double kPi = 3.14159265358979323846;
using Joints = Eigen::Matrix<double, 7, 1>;
using Jacobian = Eigen::Matrix<double, 6, 7>;
using Twist = Eigen::Matrix<double, 6, 1>;
using Pose = Eigen::Isometry3d;
inline const Joints kLower = (Joints() << -2.8973, -1.7628, -2.8973, -3.0718,
                             -2.8973, -0.0175, -2.8973).finished();
inline const Joints kUpper = (Joints() << 2.8973, 1.7628, 2.8973, -0.0698,
                             2.8973, 3.7525, 2.8973).finished();
inline const Joints kHome = (Joints() << 0., -kPi / 4., 0., -3. * kPi / 4.,
                            0., kPi / 2., kPi / 4.).finished();

struct Kinematics {
  Pose tool{Pose::Identity()};
  Jacobian jacobian{Jacobian::Zero()};
  std::array<Pose, 8> links{};
};

// URDF joint origins for the original Panda. Hardware uses libfranka's live model.
class PandaModel {
 public:
  explicit PandaModel(double tool_offset = 0.1034);
  Kinematics evaluate(const Joints& q) const;
 private:
  double tool_offset_;
};

Pose from_xyz_rpy(const Eigen::Vector3d& xyz, const Eigen::Vector3d& rpy);
Twist pose_error(const Pose& current, const Pose& goal);
bool within_limits(const Joints& q, double margin = 0.0);
}  // namespace fer
