#ifndef FER_MODEL_PANDA_MODEL_HPP
#define FER_MODEL_PANDA_MODEL_HPP

#include <array>
#include <rml/RML.h>
#include <Eigen/Geometry>

namespace fer {

constexpr std::size_t ARM_JOINTS = 7;
constexpr double PI = 3.14159265358979323846;

using JointVector = Eigen::Matrix<double, 7, 1>;
using JointMatrix = Eigen::Matrix<double, 7, 7>;
using JacobianMatrix = Eigen::Matrix<double, 6, 7>;
using CartesianVector = Eigen::Matrix<double, 6, 1>;
using Pose = Eigen::Isometry3d;

const JointVector& JointLimitsMin();
const JointVector& JointLimitsMax();
const JointVector& HomeJointPosition();
bool JointsWithinLimits(const JointVector& jointPosition, double margin = 0.0);
Pose MakePose(const Eigen::Vector3d& position, const Eigen::Vector3d& orientationRPY);
CartesianVector CartesianError(const Pose& currentFrame, const Pose& goalFrame);

struct KinematicData {
    Pose toolFrame = Pose::Identity();
    JacobianMatrix toolJacobian = JacobianMatrix::Zero();
    std::array<Pose, 8> jointFrames;
};

class PandaModel {
public:
    explicit PandaModel(double toolOffset = 0.1034);

    KinematicData ComputeKinematics(const JointVector& jointPosition) const;
    Pose FlangeToTool() const;
    static const std::array<Pose, ARM_JOINTS>& JointOrigins();

private:
    double toolOffset_;
};

class FrankaArmModel : public rml::ArmModel {
public:
    explicit FrankaArmModel(const std::string& armID = "Panda");
    void SetToolFrame(const Pose& flangeToTool);
};

}

#endif
