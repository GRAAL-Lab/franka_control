#include "fer_model/franka_arm_model.hpp"

#include <cmath>
#include <stdexcept>

namespace fer {

const JointVector& JointLimitsMin()
{
    static const JointVector limits
        = (JointVector() << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973).finished();
    return limits;
}

const JointVector& JointLimitsMax()
{
    static const JointVector limits
        = (JointVector() << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973).finished();
    return limits;
}

const JointVector& HomeJointPosition()
{
    static const JointVector position
        = (JointVector() << 0.0, -PI / 4.0, 0.0, -3.0 * PI / 4.0, 0.0, PI / 2.0, PI / 4.0).finished();
    return position;
}

bool JointsWithinLimits(const JointVector& jointPosition, double margin)
{
    return jointPosition.allFinite() && (jointPosition.array() >= JointLimitsMin().array() + margin).all()
        && (jointPosition.array() <= JointLimitsMax().array() - margin).all();
}

Eigen::TransformationMatrix MakeTransformation(
    const Eigen::Vector3d& position, const rml::EulerRPY& orientationRPY)
{
    Eigen::TransformationMatrix frame;
    frame.TranslationVector(position);
    frame.RotationMatrix(orientationRPY.ToRotationMatrix());
    return frame;
}

namespace {

const Eigen::TransformationMatrix& JointToFlange()
{
    static const auto frame = MakeTransformation({ 0.0, 0.0, 0.107 }, {});
    return frame;
}

const std::array<Eigen::TransformationMatrix, ARM_JOINTS>& JointOrigins()
{
    static const std::array<Eigen::TransformationMatrix, ARM_JOINTS> origins = { {
        MakeTransformation({ 0.0, 0.0, 0.333 }, {}),
        MakeTransformation({ 0.0, 0.0, 0.0 }, { -PI / 2.0, 0.0, 0.0 }),
        MakeTransformation({ 0.0, -0.316, 0.0 }, { PI / 2.0, 0.0, 0.0 }),
        MakeTransformation({ 0.0825, 0.0, 0.0 }, { PI / 2.0, 0.0, 0.0 }),
        MakeTransformation({ -0.0825, 0.384, 0.0 }, { -PI / 2.0, 0.0, 0.0 }),
        MakeTransformation({ 0.0, 0.0, 0.0 }, { PI / 2.0, 0.0, 0.0 }),
        MakeTransformation({ 0.088, 0.0, 0.0 }, { PI / 2.0, 0.0, 0.0 }) } };
    return origins;
}

}

FrankaArmModel::FrankaArmModel(const std::string& armID, double toolOffset)
    : rml::ArmModel(armID)
{
    if (!std::isfinite(toolOffset) || toolOffset < 0.0) {
        throw std::invalid_argument("toolOffset must be finite and nonnegative");
    }
    for (std::size_t joint = 0; joint < ARM_JOINTS; ++joint) {
        AddJointLink(rml::JointType::Revolute, Eigen::Vector3d::UnitZ(), JointOrigins()[joint],
            JointLimitsMin()[joint], JointLimitsMax()[joint]);
    }
    JointsPosition(HomeJointPosition());
    JointsVelocity(JointVector::Zero());
    SetToolFrame(MakeTransformation({ 0.0, 0.0, toolOffset }, { 0.0, 0.0, -PI / 4.0 }));
}

void FrankaArmModel::SetToolFrame(const Eigen::TransformationMatrix& flangeToTool)
{
    Eigen::TransformationMatrix jointToTool = JointToFlange() * flangeToTool;
    AttachRigidBodyFrame("Tool", ID() + rml::FrameID::Joint + "6", jointToTool);
    flangeToTool_ = flangeToTool;
}

}
