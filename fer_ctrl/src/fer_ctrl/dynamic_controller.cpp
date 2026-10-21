#include "fer_ctrl/dynamic_controller.hpp"

#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <libconfig.h++>
#include <stdexcept>

namespace fer {
namespace {

JointVector ReadJointParameter(const libconfig::Setting& root, const char* name)
{
    const auto& values = root[name];
    if (values.getLength() != static_cast<int>(ARM_JOINTS)) {
        throw std::invalid_argument(std::string(name) + " must contain seven values");
    }
    JointVector parameter;
    for (std::size_t joint = 0; joint < ARM_JOINTS; ++joint) {
        parameter[joint] = static_cast<double>(values[joint]);
    }
    return parameter;
}

}

void DCLConfiguration::LoadConfiguration(const std::string& fileName)
{
    libconfig::Config configuration;
    configuration.setAutoConvert(true);
    configuration.readFile(fileName.c_str());
    const auto& root = configuration.getRoot();

    controlMode = root["controlMode"].c_str();
    loopRate = root["loopRate"];
    feedbackTimeout = root["feedbackTimeout"];
    commandTimeout = root["commandTimeout"];
    maxJointVelocity = root["maxJointVelocity"];
    maxJointAcceleration = root["maxJointAcceleration"];
    maxJointJerk = root["maxJointJerk"];
    referenceVelocityGain = root["referenceVelocityGain"];
    maxTrackingError = root["maxTrackingError"];
    jointLimitMargin = root["jointLimitMargin"];
    positionGain = ReadJointParameter(root, "positionGain");
    velocityGain = ReadJointParameter(root, "velocityGain");
    torqueLimit = ReadJointParameter(root, "torqueLimit");
    torqueRateLimit = ReadJointParameter(root, "torqueRateLimit");
    Validate();
}

void DCLConfiguration::Validate() const
{
    for (double value : { loopRate, feedbackTimeout, commandTimeout, maxJointVelocity, maxJointAcceleration,
             maxJointJerk, referenceVelocityGain, maxTrackingError, jointLimitMargin }) {
        if (!std::isfinite(value) || value <= 0.0) {
            throw std::invalid_argument("Dynamic controller parameters must be finite and positive");
        }
    }
    JointVector robotTorqueLimit;
    robotTorqueLimit << 87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0;
    if ((controlMode != "ComputedTorque" && controlMode != "JointImpedance") || loopRate < 100.0
        || loopRate > 1000.0 || maxJointVelocity > 2.175 || !positionGain.allFinite()
        || !velocityGain.allFinite() || !torqueLimit.allFinite() || !torqueRateLimit.allFinite()
        || (positionGain.array() <= 0.0).any() || (velocityGain.array() <= 0.0).any()
        || (torqueLimit.array() <= 0.0).any() || (torqueLimit.array() > robotTorqueLimit.array()).any()
        || (torqueRateLimit.array() <= 0.0).any() || (torqueRateLimit.array() > 1000.0).any()
        || jointLimitMargin >= 0.5 * (JointLimitsMax() - JointLimitsMin()).minCoeff()) {
        throw std::invalid_argument("Invalid dynamic control mode, gains or limits");
    }
}

JointReferenceGenerator::JointReferenceGenerator(const DCLConfiguration& configuration)
    : conf_(configuration)
{
    conf_.Validate();
}

void JointReferenceGenerator::Reset(const JointVector& jointPosition)
{
    reference_.position = jointPosition;
    reference_.velocity.setZero();
    reference_.acceleration.setZero();
}

void JointReferenceGenerator::SetReference(const JointReference& reference)
{
    reference_ = reference;
}

const JointReference& JointReferenceGenerator::GetReference() const
{
    return reference_;
}

const JointReference& JointReferenceGenerator::Run(
    const JointVector& requestedVelocity, const JointVector& measuredPosition, double sampleTime)
{
    if (!std::isfinite(sampleTime) || sampleTime <= 0.0) {
        return reference_;
    }
    sampleTime = std::min(sampleTime, 0.02);
    for (std::size_t joint = 0; joint < ARM_JOINTS; ++joint) {
        double targetVelocity
            = std::clamp(requestedVelocity[joint], -conf_.maxJointVelocity, conf_.maxJointVelocity);
        double targetAcceleration
            = conf_.referenceVelocityGain * (targetVelocity - reference_.velocity[joint]);
        targetAcceleration
            = std::clamp(targetAcceleration, -conf_.maxJointAcceleration, conf_.maxJointAcceleration);
        double accelerationChange = targetAcceleration - reference_.acceleration[joint];
        reference_.acceleration[joint] += std::clamp(
            accelerationChange, -conf_.maxJointJerk * sampleTime, conf_.maxJointJerk * sampleTime);
        double previousVelocity = reference_.velocity[joint];
        reference_.velocity[joint] += reference_.acceleration[joint] * sampleTime;
        reference_.velocity[joint]
            = std::clamp(reference_.velocity[joint], -conf_.maxJointVelocity, conf_.maxJointVelocity);
        reference_.position[joint] += 0.5 * (previousVelocity + reference_.velocity[joint]) * sampleTime;

        double lower = std::max(JointLimitsMin()[joint] + conf_.jointLimitMargin,
            measuredPosition[joint] - conf_.maxTrackingError);
        double upper = std::min(JointLimitsMax()[joint] - conf_.jointLimitMargin,
            measuredPosition[joint] + conf_.maxTrackingError);
        // A measured joint outside the software margin must still be allowed to retreat.
        lower = std::min(lower, measuredPosition[joint]);
        upper = std::max(upper, measuredPosition[joint]);
        reference_.position[joint] = std::clamp(reference_.position[joint], lower, upper);
    }
    return reference_;
}

DynamicController::DynamicController(const DCLConfiguration& configuration)
    : conf_(configuration)
{
    conf_.Validate();
}

bool DynamicController::ComputeTorques(const JointReference& reference, const JointVector& jointPosition,
    const JointVector& jointVelocity, const JointMatrix& massMatrix, const JointVector& coriolis,
    const JointVector& previousTorque, double sampleTime, JointVector& commandedTorque) const
{
    commandedTorque.setZero();
    if (!jointPosition.allFinite() || !jointVelocity.allFinite() || !massMatrix.allFinite()
        || !coriolis.allFinite() || !previousTorque.allFinite() || !reference.position.allFinite()
        || !reference.velocity.allFinite() || !reference.acceleration.allFinite()
        || !std::isfinite(sampleTime) || sampleTime <= 0.0
        || (previousTorque.cwiseAbs().array() > conf_.torqueLimit.array() + 1e-6).any()
        || (massMatrix - massMatrix.transpose()).norm() > 1e-6) {
        return false;
    }
    Eigen::LDLT<JointMatrix> massFactorization(massMatrix);
    if (massFactorization.info() != Eigen::Success || (massFactorization.vectorD().array() <= 0.0).any()) {
        return false;
    }

    JointVector positionError = reference.position - jointPosition;
    positionError = positionError.cwiseMax(-conf_.maxTrackingError).cwiseMin(conf_.maxTrackingError);
    JointVector velocityError = reference.velocity - jointVelocity;
    JointVector feedback
        = conf_.positionGain.cwiseProduct(positionError) + conf_.velocityGain.cwiseProduct(velocityError);
    if (conf_.controlMode == "ComputedTorque") {
        commandedTorque = massMatrix * (reference.acceleration + feedback) + coriolis;
    } else {
        commandedTorque = feedback + coriolis;
    }
    if (!commandedTorque.allFinite()) {
        return false;
    }

    // libfranka compensates gravity internally. Adding model.gravity() here would count it twice.
    for (std::size_t joint = 0; joint < ARM_JOINTS; ++joint) {
        double limitedTorque
            = std::clamp(commandedTorque[joint], -conf_.torqueLimit[joint], conf_.torqueLimit[joint]);
        double maximumChange = conf_.torqueRateLimit[joint] * sampleTime;
        commandedTorque[joint] = std::clamp(
            limitedTorque, previousTorque[joint] - maximumChange, previousTorque[joint] + maximumChange);
    }
    return true;
}

}
