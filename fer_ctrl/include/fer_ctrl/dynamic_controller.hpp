/** @file dynamic_controller.hpp */

#ifndef FER_CTRL_DYNAMIC_CONTROLLER_HPP
#define FER_CTRL_DYNAMIC_CONTROLLER_HPP

#include "fer_model/franka_arm_model.hpp"
#include <string>

namespace fer {

/**
 * @brief Shared reference-generation and torque-control settings.
 * @details Rates use Hz, timeouts seconds, joint motion rad/s, rad/s^2 and rad/s^3,
 * and torque limits Nm and Nm/s. Gains follow the selected controlMode.
 */
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

    /**
     * @brief Read the dynamic controller libconfig file and validate every setting.
     * @param fileName Path to dcl_fer.conf.
     * @throws std::exception If reading, parsing or validation fails.
     */
    void LoadConfiguration(const std::string& fileName);
    /**
     * @brief Check the control mode, finite positive gains and supported Panda limits.
     * @throws std::invalid_argument If a setting is outside the supported range.
     */
    void Validate() const;
};

/** @brief Joint position, velocity and acceleration references in rad, rad/s and rad/s^2. */
struct JointReference {
    JointVector position = HomeJointPosition();
    JointVector velocity = JointVector::Zero();
    JointVector acceleration = JointVector::Zero();
};

/** @brief Integrates velocity requests with acceleration, jerk, tracking and joint-position limits. */
class JointReferenceGenerator {
public:
    /**
     * @brief Create a reference generator using a validated copy of the settings.
     * @param configuration Dynamic limits and reference feedback gain.
     * @throws std::invalid_argument If configuration is invalid.
     */
    explicit JointReferenceGenerator(const DCLConfiguration& configuration);
    /** @brief Reset to the measured joint position in radians with zero velocity and acceleration. */
    void Reset(const JointVector& jointPosition);
    /** @brief Seed all reference components, preserving continuity when starting a controlled stop. */
    void SetReference(const JointReference& reference);
    /**
     * @brief Integrate one reference step with configured motion and tracking bounds.
     * @param requestedVelocity Requested joint velocities in rad/s.
     * @param measuredPosition Measured joint positions in radians.
     * @param sampleTime Elapsed controller time in seconds; capped at 0.02 s.
     * @return Stored reference after this step; unchanged for nonpositive or nonfinite time.
     */
    const JointReference& Run(
        const JointVector& requestedVelocity, const JointVector& measuredPosition, double sampleTime);
    /** @brief Return the stored reference without advancing the generator. */
    const JointReference& GetReference() const;

private:
    DCLConfiguration configuration_;
    JointReference reference_;
};

/** @brief Computed-torque or joint-impedance control with absolute torque and torque-rate limits. */
class DynamicController {
public:
    /**
     * @brief Create a torque controller using a validated copy of the settings.
     * @param configuration Control mode, gains and limits.
     * @throws std::invalid_argument If configuration is invalid.
     */
    explicit DynamicController(const DCLConfiguration& configuration);

    /**
     * @brief Compute a bounded torque command without adding gravity compensation.
     * @param reference Desired joint position, velocity and acceleration.
     * @param jointPosition Measured joint positions in radians.
     * @param jointVelocity Measured joint velocities in rad/s.
     * @param massMatrix Symmetric positive-definite joint inertia matrix.
     * @param coriolis Coriolis/centrifugal torque vector in Nm.
     * @param previousTorque Previously commanded torque in Nm, used for rate limiting.
     * @param sampleTime Elapsed control time in seconds; must be finite and positive.
     * @param[out] commandedTorque Output torques in Nm; use only when the return value is true.
     * @return False for invalid inputs, dynamics or computed torques.
     * @note libfranka compensates gravity internally.
     */
    bool ComputeTorques(const JointReference& reference, const JointVector& jointPosition,
        const JointVector& jointVelocity, const JointMatrix& massMatrix, const JointVector& coriolis,
        const JointVector& previousTorque, double sampleTime, JointVector& commandedTorque) const;

private:
    DCLConfiguration configuration_;
};

}

#endif
