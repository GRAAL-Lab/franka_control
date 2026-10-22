#ifndef FER_CTRL_DYNAMIC_CONTROLLER_HPP
#define FER_CTRL_DYNAMIC_CONTROLLER_HPP

#include "fer_model/franka_arm_model.hpp"
#include <string>

namespace fer {

struct DCLConfiguration {
    std::string controlMode = "ComputedTorque";
    double loopRate = 500.0;
    double feedbackTimeout = 0.2;
    double commandTimeout = 0.2;
    double maxJointVelocity = 0.35;
    double maxJointAcceleration = 0.6;
    double maxJointJerk = 5.0;
    double referenceVelocityGain = 20.0;
    double maxTrackingError = 0.1;
    double jointLimitMargin = 0.05;
    JointVector positionGain = JointVector::Constant(40.0);
    JointVector velocityGain = JointVector::Constant(12.0);
    JointVector torqueLimit = (JointVector() << 30.0, 30.0, 30.0, 30.0, 8.0, 8.0, 8.0).finished();
    JointVector torqueRateLimit = JointVector::Constant(200.0);

    void LoadConfiguration(const std::string& fileName);
    void Validate() const;
};

struct JointReference {
    JointVector position = HomeJointPosition();
    JointVector velocity = JointVector::Zero();
    JointVector acceleration = JointVector::Zero();
};

class JointReferenceGenerator {
public:
    explicit JointReferenceGenerator(const DCLConfiguration& configuration);
    void Reset(const JointVector& jointPosition);
    void SetReference(const JointReference& reference);
    const JointReference& Run(
        const JointVector& requestedVelocity, const JointVector& measuredPosition, double sampleTime);
    const JointReference& GetReference() const;

private:
    DCLConfiguration conf_;
    JointReference reference_;
};

class DynamicController {
public:
    explicit DynamicController(const DCLConfiguration& configuration);

    bool ComputeTorques(const JointReference& reference, const JointVector& jointPosition,
        const JointVector& jointVelocity, const JointMatrix& massMatrix, const JointVector& coriolis,
        const JointVector& previousTorque, double sampleTime, JointVector& commandedTorque) const;

private:
    DCLConfiguration conf_;
};

}

#endif
