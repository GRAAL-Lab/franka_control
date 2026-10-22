#ifndef FER_MODEL_FRANKA_ARM_MODEL_HPP
#define FER_MODEL_FRANKA_ARM_MODEL_HPP

#include <array>
#include <rml/RML.h>

namespace fer {

constexpr std::size_t ARM_JOINTS = 7;
constexpr double PI = 3.14159265358979323846;
constexpr double DEFAULT_TOOL_OFFSET = 0.1034;

using JointVector = Eigen::Matrix<double, 7, 1>;
using JointMatrix = Eigen::Matrix<double, 7, 7>;
using JacobianMatrix = Eigen::Matrix<double, 6, 7>;

const JointVector& JointLimitsMin();
const JointVector& JointLimitsMax();
const JointVector& HomeJointPosition();
bool JointsWithinLimits(const JointVector& jointPosition, double margin = 0.0);
Eigen::TransformationMatrix MakeTransformation(
    const Eigen::Vector3d& position, const rml::EulerRPY& orientationRPY);

class FrankaArmModel : public rml::ArmModel {
public:
    explicit FrankaArmModel(const std::string& armID = "Panda", double toolOffset = DEFAULT_TOOL_OFFSET);
    void SetToolFrame(const Eigen::TransformationMatrix& flangeToTool);
    const Eigen::TransformationMatrix& FlangeToTool() const { return flangeToTool_; }
    std::string ToolFrameID() const { return ID() + "_Tool"; }

private:
    Eigen::TransformationMatrix flangeToTool_;
};

}

#endif
