#include "fer_model/panda_model.hpp"

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

Pose MakePose(const Eigen::Vector3d& position, const Eigen::Vector3d& orientationRPY)
{
    Pose frame = Pose::Identity();
    frame.translation() = position;
    frame.linear() = (Eigen::AngleAxisd(orientationRPY.z(), Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(orientationRPY.y(), Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(orientationRPY.x(), Eigen::Vector3d::UnitX()))
                         .toRotationMatrix();
    return frame;
}

CartesianVector CartesianError(const Pose& currentFrame, const Pose& goalFrame)
{
    CartesianVector error;
    error.head<3>() = goalFrame.translation() - currentFrame.translation();
    Eigen::AngleAxisd orientationError(goalFrame.linear() * currentFrame.linear().transpose());
    error.tail<3>() = orientationError.axis() * orientationError.angle();
    return error;
}

PandaModel::PandaModel(double toolOffset)
    : toolOffset_(toolOffset)
{
    if (!std::isfinite(toolOffset_) || toolOffset_ < 0.0) {
        throw std::invalid_argument("toolOffset must be finite and nonnegative");
    }
}

const std::array<Pose, ARM_JOINTS>& PandaModel::JointOrigins()
{
    static const std::array<Pose, ARM_JOINTS> origins = { { MakePose({ 0.0, 0.0, 0.333 }, { 0.0, 0.0, 0.0 }),
        MakePose({ 0.0, 0.0, 0.0 }, { -PI / 2.0, 0.0, 0.0 }),
        MakePose({ 0.0, -0.316, 0.0 }, { PI / 2.0, 0.0, 0.0 }),
        MakePose({ 0.0825, 0.0, 0.0 }, { PI / 2.0, 0.0, 0.0 }),
        MakePose({ -0.0825, 0.384, 0.0 }, { -PI / 2.0, 0.0, 0.0 }),
        MakePose({ 0.0, 0.0, 0.0 }, { PI / 2.0, 0.0, 0.0 }),
        MakePose({ 0.088, 0.0, 0.0 }, { PI / 2.0, 0.0, 0.0 }) } };
    return origins;
}

Pose PandaModel::FlangeToTool() const
{
    return MakePose({ 0.0, 0.0, toolOffset_ }, { 0.0, 0.0, -PI / 4.0 });
}

KinematicData PandaModel::ComputeKinematics(const JointVector& jointPosition) const
{
    KinematicData data;
    data.jointFrames[0] = Pose::Identity();
    std::array<Eigen::Vector3d, ARM_JOINTS> jointPositions;
    std::array<Eigen::Vector3d, ARM_JOINTS> jointAxes;
    Pose baseToJoint = Pose::Identity();

    for (std::size_t joint = 0; joint < ARM_JOINTS; ++joint) {
        baseToJoint = baseToJoint * JointOrigins()[joint];
        jointPositions[joint] = baseToJoint.translation();
        jointAxes[joint] = baseToJoint.linear().col(2);
        baseToJoint.rotate(Eigen::AngleAxisd(jointPosition[joint], Eigen::Vector3d::UnitZ()));
        data.jointFrames[joint + 1] = baseToJoint;
    }

    Pose jointToFlange = MakePose({ 0.0, 0.0, 0.107 }, Eigen::Vector3d::Zero());
    data.toolFrame = baseToJoint * jointToFlange * FlangeToTool();
    for (std::size_t joint = 0; joint < ARM_JOINTS; ++joint) {
        data.toolJacobian.block<3, 1>(0, joint)
            = jointAxes[joint].cross(data.toolFrame.translation() - jointPositions[joint]);
        data.toolJacobian.block<3, 1>(3, joint) = jointAxes[joint];
    }
    return data;
}

FrankaArmModel::FrankaArmModel(const std::string& armID)
    : rml::ArmModel(armID)
{
    for (std::size_t joint = 0; joint < ARM_JOINTS; ++joint) {
        Eigen::TransformationMatrix jointOrigin(PandaModel::JointOrigins()[joint].matrix());
        AddJointLink(rml::JointType::Revolute, Eigen::Vector3d::UnitZ(), jointOrigin, JointLimitsMin()[joint],
            JointLimitsMax()[joint]);
    }
    JointsPosition(HomeJointPosition());
    JointsVelocity(JointVector::Zero());
    SetToolFrame(PandaModel().FlangeToTool());
}

void FrankaArmModel::SetToolFrame(const Pose& flangeToTool)
{
    Pose jointToTool = MakePose({ 0.0, 0.0, 0.107 }, Eigen::Vector3d::Zero()) * flangeToTool;
    AttachRigidBodyFrame(
        "Tool", ID() + rml::FrameID::Joint + "6", Eigen::TransformationMatrix(jointToTool.matrix()));
}

}
