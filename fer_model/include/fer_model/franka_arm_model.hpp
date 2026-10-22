/** @file franka_arm_model.hpp */

#ifndef FER_MODEL_FRANKA_ARM_MODEL_HPP
#define FER_MODEL_FRANKA_ARM_MODEL_HPP

#include <array>
#include <rml/RML.h>

namespace fer {

constexpr std::size_t ARM_JOINTS = 7;
constexpr double PI = 3.14159265358979323846;
/** @brief Default flange-to-tool translation along flange Z, in metres. */
constexpr double DEFAULT_TOOL_OFFSET = 0.1034;

using JointVector = Eigen::Matrix<double, 7, 1>;
using JointMatrix = Eigen::Matrix<double, 7, 7>;
using JacobianMatrix = Eigen::Matrix<double, 6, 7>;

/** @brief Return the seven Panda lower joint limits in radians. */
const JointVector& JointLimitsMin();
/** @brief Return the seven Panda upper joint limits in radians. */
const JointVector& JointLimitsMax();
/** @brief Return the shared home posture in radians, ordered by joint 1 through 7. */
const JointVector& HomeJointPosition();
/**
 * @brief Check finite joint positions against the Panda position limits.
 * @param jointPosition Joint positions in radians.
 * @param margin Inward offset from each limit in radians.
 * @return True when every joint lies within the inclusive bounds.
 */
bool JointsWithinLimits(const JointVector& jointPosition, double margin = 0.0);
/**
 * @brief Construct an RML homogeneous transformation.
 * @param position Translation in metres, expressed in the parent frame.
 * @param orientationRPY Roll, pitch and yaw in radians.
 * @return Transformation from the child frame to the parent frame.
 */
Eigen::TransformationMatrix MakeTransformation(
    const Eigen::Vector3d& position, const rml::EulerRPY& orientationRPY);

/** @brief Seven-joint Panda kinematics using RML frames and Jacobians. */
class FrankaArmModel : public rml::ArmModel {
public:
    /**
     * @brief Create the Panda arm at the home posture with its default tool orientation.
     * @param armID Prefix for the RML arm and attached frame identifiers.
     * @param toolOffset Flange-to-tool Z translation in metres; shared default is DEFAULT_TOOL_OFFSET.
     * @throws std::invalid_argument If toolOffset is negative or nonfinite.
     */
    explicit FrankaArmModel(const std::string& armID = "Panda", double toolOffset = DEFAULT_TOOL_OFFSET);
    /**
     * @brief Replace the flange-to-tool transform and update the attached RML tool frame.
     * @param flangeToTool Transform expressed in the flange frame.
     */
    void SetToolFrame(const Eigen::TransformationMatrix& flangeToTool);
    /** @brief Return the currently configured flange-to-tool transform. */
    const Eigen::TransformationMatrix& FlangeToTool() const { return flangeToTool_; }
    /** @brief Return the attached tool frame identifier used by RML lookups. */
    std::string ToolFrameID() const { return ID() + "_Tool"; }

private:
    Eigen::TransformationMatrix flangeToTool_;
};

}

#endif
