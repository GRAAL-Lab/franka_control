#include "fer_model/panda_model.hpp"

#include <cmath>
#include <stdexcept>

namespace fer {
Pose from_xyz_rpy(const Eigen::Vector3d& xyz, const Eigen::Vector3d& rpy) {
  Pose pose = Pose::Identity();
  pose.translation() = xyz;
  pose.linear() = (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
                   Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
                   Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
  return pose;
}

Twist pose_error(const Pose& current, const Pose& goal) {
  Twist result;
  result.head<3>() = goal.translation() - current.translation();
  Eigen::AngleAxisd rotation(goal.linear() * current.linear().transpose());
  result.tail<3>() = rotation.axis() * rotation.angle();
  return result;
}

bool within_limits(const Joints& q, double margin) {
  return q.allFinite() && (q.array() >= kLower.array() + margin).all() &&
         (q.array() <= kUpper.array() - margin).all();
}

PandaModel::PandaModel(double tool_offset) : tool_offset_(tool_offset) {
  if (!std::isfinite(tool_offset) || tool_offset < 0.0) {
    throw std::invalid_argument("tool_offset must be finite and nonnegative");
  }
}

Kinematics PandaModel::evaluate(const Joints& q) const {
  const std::array<Eigen::Vector3d, 7> xyz{{{0., 0., .333}, {0., 0., 0.},
      {0., -.316, 0.}, {.0825, 0., 0.}, {-.0825, .384, 0.},
      {0., 0., 0.}, {.088, 0., 0.}}};
  const std::array<double, 7> roll{{0., -kPi / 2., kPi / 2., kPi / 2.,
                                  -kPi / 2., kPi / 2., kPi / 2.}};
  Kinematics result;
  result.links[0] = Pose::Identity();
  std::array<Eigen::Vector3d, 7> origins, axes;
  Pose transform = Pose::Identity();
  for (std::size_t i = 0; i < kJoints; ++i) {
    transform = transform * from_xyz_rpy(xyz[i], {roll[i], 0., 0.});
    origins[i] = transform.translation();
    axes[i] = transform.linear().col(2);
    transform.rotate(Eigen::AngleAxisd(q[i], Eigen::Vector3d::UnitZ()));
    result.links[i + 1] = transform;
  }
  result.tool = transform * from_xyz_rpy({0., 0., .107 + tool_offset_}, {0., 0., -kPi / 4.});
  for (std::size_t i = 0; i < kJoints; ++i) {
    result.jacobian.block<3, 1>(0, i) = axes[i].cross(result.tool.translation() - origins[i]);
    result.jacobian.block<3, 1>(3, i) = axes[i];
  }
  return result;
}
}  // namespace fer
