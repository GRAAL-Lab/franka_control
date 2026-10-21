#ifndef FER_MSGS_CONVERSIONS_HPP
#define FER_MSGS_CONVERSIONS_HPP

#include "fer_model/panda_model.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <chrono>
#include <stdexcept>
#include <vector>

namespace fer {

inline double SteadyTime()
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

inline geometry_msgs::msg::Pose PoseToMessage(const Pose& frame)
{
    geometry_msgs::msg::Pose message;
    message.position.x = frame.translation().x();
    message.position.y = frame.translation().y();
    message.position.z = frame.translation().z();
    Eigen::Quaterniond orientation(frame.linear());
    message.orientation.x = orientation.x();
    message.orientation.y = orientation.y();
    message.orientation.z = orientation.z();
    message.orientation.w = orientation.w();
    return message;
}

inline Pose MessageToPose(const geometry_msgs::msg::Pose& message)
{
    Eigen::Quaterniond orientation(
        message.orientation.w, message.orientation.x, message.orientation.y, message.orientation.z);
    if (!orientation.coeffs().allFinite() || std::abs(orientation.norm() - 1.0) > 1e-5) {
        throw std::invalid_argument("Invalid feedback quaternion");
    }
    Pose frame = Pose::Identity();
    frame.translation() = Eigen::Vector3d(message.position.x, message.position.y, message.position.z);
    frame.linear() = orientation.toRotationMatrix();
    return frame;
}

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
