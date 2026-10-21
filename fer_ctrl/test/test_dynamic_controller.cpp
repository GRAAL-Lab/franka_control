#include "fer_ctrl/dynamic_controller.hpp"
#include <gtest/gtest.h>
#include <limits>

TEST(DynamicController, LoadsConfigurationFile)
{
    fer::DCLConfiguration configuration;
    configuration.LoadConfiguration(FER_DCL_CONFIG);
    EXPECT_EQ(configuration.controlMode, "ComputedTorque");
    EXPECT_EQ(configuration.positionGain.size(), 7);
    EXPECT_DOUBLE_EQ(configuration.torqueLimit[6], 8.0);
}

TEST(DynamicController, ComputesInertiaAndCoriolisCompensationWithoutAddingGravity)
{
    fer::DCLConfiguration configuration;
    fer::DynamicController controller(configuration);
    fer::JointReference reference;
    reference.position = fer::HomeJointPosition();
    reference.position[0] += 0.01;
    fer::JointVector torque;
    ASSERT_TRUE(controller.ComputeTorques(reference, fer::HomeJointPosition(), fer::JointVector::Zero(),
        2.0 * fer::JointMatrix::Identity(), fer::JointVector::Constant(0.1), fer::JointVector::Zero(), 0.01,
        torque));
    EXPECT_NEAR(torque[0], 0.9, 1e-10);
    EXPECT_NEAR(torque[1], 0.1, 1e-10);
}

TEST(DynamicController, SupportsJointImpedanceMode)
{
    fer::DCLConfiguration configuration;
    configuration.controlMode = "JointImpedance";
    fer::DynamicController controller(configuration);
    fer::JointReference reference;
    reference.position[0] += 0.01;
    fer::JointVector torque;
    ASSERT_TRUE(controller.ComputeTorques(reference, fer::HomeJointPosition(), fer::JointVector::Zero(),
        2.0 * fer::JointMatrix::Identity(), fer::JointVector::Constant(0.1), fer::JointVector::Zero(), 0.01,
        torque));
    EXPECT_NEAR(torque[0], 0.5, 1e-10);
}

TEST(DynamicController, EnforcesTorqueAndTorqueRateLimits)
{
    fer::DCLConfiguration configuration;
    fer::DynamicController controller(configuration);
    fer::JointReference reference;
    reference.position = fer::HomeJointPosition() + fer::JointVector::Constant(0.1);
    fer::JointVector torque;
    ASSERT_TRUE(controller.ComputeTorques(reference, fer::HomeJointPosition(), fer::JointVector::Zero(),
        100.0 * fer::JointMatrix::Identity(), fer::JointVector::Zero(), fer::JointVector::Zero(), 1.0,
        torque));
    EXPECT_LE((torque.cwiseAbs() - configuration.torqueLimit).maxCoeff(), 1e-9);
    EXPECT_NEAR(torque[0], 30.0, 1e-9);
    ASSERT_TRUE(controller.ComputeTorques(reference, fer::HomeJointPosition(), fer::JointVector::Zero(),
        100.0 * fer::JointMatrix::Identity(), fer::JointVector::Zero(), fer::JointVector::Zero(), 0.001,
        torque));
    EXPECT_LE(torque.cwiseAbs().maxCoeff(), 0.2000001);
}

TEST(DynamicController, RejectsInvalidStateAndNonPhysicalInertia)
{
    fer::DynamicController controller { fer::DCLConfiguration() };
    fer::JointReference reference;
    fer::JointVector torque;
    EXPECT_FALSE(controller.ComputeTorques(reference, fer::HomeJointPosition(), fer::JointVector::Zero(),
        -fer::JointMatrix::Identity(), fer::JointVector::Zero(), fer::JointVector::Zero(), 0.001, torque));
    fer::JointVector invalidPosition = fer::HomeJointPosition();
    invalidPosition[0] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(controller.ComputeTorques(reference, invalidPosition, fer::JointVector::Zero(),
        fer::JointMatrix::Identity(), fer::JointVector::Zero(), fer::JointVector::Zero(), 0.001, torque));
    EXPECT_TRUE(torque.isZero());
}

TEST(DynamicController, RejectsInvalidConfiguration)
{
    fer::DCLConfiguration configuration;
    configuration.controlMode = "unknown";
    EXPECT_THROW(fer::DynamicController controller(configuration), std::invalid_argument);
    configuration = {};
    configuration.torqueLimit[6] = 100.0;
    EXPECT_THROW(fer::DynamicController controller(configuration), std::invalid_argument);
}

TEST(JointReferenceGenerator, BoundsVelocityAccelerationJerkAndTrackingError)
{
    fer::DCLConfiguration configuration;
    fer::JointReferenceGenerator generator(configuration);
    generator.Reset(fer::HomeJointPosition());
    fer::JointVector previousAcceleration = fer::JointVector::Zero();
    for (int step = 0; step < 1000; ++step) {
        const auto& reference
            = generator.Run(fer::JointVector::Constant(0.25), fer::HomeJointPosition(), 0.002);
        EXPECT_LE(reference.velocity.cwiseAbs().maxCoeff(), configuration.maxJointVelocity + 1e-9);
        EXPECT_LE(reference.acceleration.cwiseAbs().maxCoeff(), configuration.maxJointAcceleration + 1e-9);
        EXPECT_LE((reference.acceleration - previousAcceleration).cwiseAbs().maxCoeff(),
            configuration.maxJointJerk * 0.002 + 1e-9);
        EXPECT_LE((reference.position - fer::HomeJointPosition()).cwiseAbs().maxCoeff(),
            configuration.maxTrackingError + 1e-9);
        previousAcceleration = reference.acceleration;
    }
    for (int step = 0; step < 1000; ++step) {
        generator.Run(fer::JointVector::Zero(), fer::HomeJointPosition(), 0.002);
    }
    EXPECT_LT(generator.GetReference().velocity.norm(), 1e-5);
}

TEST(DynamicController, TracksVelocityAndSettlesAfterStop)
{
    fer::DCLConfiguration configuration;
    fer::DynamicController controller(configuration);
    fer::JointReferenceGenerator generator(configuration);
    fer::JointVector position = fer::HomeJointPosition();
    fer::JointVector velocity = fer::JointVector::Zero();
    fer::JointVector previousTorque = fer::JointVector::Zero();
    generator.Reset(position);
    for (int step = 0; step < 2000; ++step) {
        fer::JointVector requestedVelocity = fer::JointVector::Zero();
        if (step < 1000) {
            requestedVelocity[0] = 0.1;
        }
        const auto& reference = generator.Run(requestedVelocity, position, 0.002);
        fer::JointVector torque;
        ASSERT_TRUE(controller.ComputeTorques(reference, position, velocity,
            2.0 * fer::JointMatrix::Identity(), fer::JointVector::Zero(), previousTorque, 0.002, torque));
        velocity += torque / 2.0 * 0.002;
        position += velocity * 0.002;
        previousTorque = torque;
    }
    EXPECT_GT(position[0], 0.15);
    EXPECT_LT(velocity.cwiseAbs().maxCoeff(), 0.001);
    EXPECT_LT((position - generator.GetReference().position).norm(), 0.001);
}
