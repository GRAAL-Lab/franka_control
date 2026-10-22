/** @file conversions.hpp */

#ifndef FER_MSGS_CONVERSIONS_HPP
#define FER_MSGS_CONVERSIONS_HPP

#include "fer_model/franka_arm_model.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <chrono>
#include <stdexcept>
#include <vector>

namespace fer {

/**
 * @brief Read monotonic wall time in seconds for hardware watchdogs.
 * @return Seconds since the steady clock epoch; unrelated to ROS simulated time.
 */
inline double SteadyTime()
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

/**
 * @brief Convert an RML transform to a ROS pose in the same parent frame.
 * @param frame Transform with translation in metres and a valid rotation.
 * @return Pose containing the translation and quaternion orientation.
 */
inline geometry_msgs::msg::Pose PoseToMessage(const Eigen::TransformationMatrix& frame)
{
    geometry_msgs::msg::Pose message;
    message.position.x = frame.TranslationVector().x();
    message.position.y = frame.TranslationVector().y();
    message.position.z = frame.TranslationVector().z();
    Eigen::Quaterniond orientation = frame.RotationMatrix().ToQuaternion();
    message.orientation.x = orientation.x();
    message.orientation.y = orientation.y();
    message.orientation.z = orientation.z();
    message.orientation.w = orientation.w();
    return message;
}

/**
 * @brief Convert a ROS pose to an RML transform in the same parent frame.
 * @param message Pose with translation in metres and a unit quaternion.
 * @return RML transformation matrix.
 * @throws std::invalid_argument If the quaternion is nonfinite or not unit length.
 * @note Callers validate translation finiteness at the feedback or command boundary.
 */
inline Eigen::TransformationMatrix MessageToPose(const geometry_msgs::msg::Pose& message)
{
    Eigen::Quaterniond orientation(
        message.orientation.w, message.orientation.x, message.orientation.y, message.orientation.z);
    if (!orientation.coeffs().allFinite() || std::abs(orientation.norm() - 1.0) > 1e-5) {
        throw std::invalid_argument("Invalid feedback quaternion");
    }
    Eigen::TransformationMatrix frame;
    frame.TranslationVector({ message.position.x, message.position.y, message.position.z });
    frame.RotationMatrix(Eigen::RotationMatrix(orientation));
    return frame;
}

/** @brief Return the canonical ROS joint names, panda_joint1 through panda_joint7. */
inline std::vector<std::string> JointNames()
{
    std::vector<std::string> names;
    for (std::size_t joint = 0; joint < ARM_JOINTS; ++joint) {
        names.push_back("panda_joint" + std::to_string(joint + 1));
    }
    return names;
}

}

#endif
