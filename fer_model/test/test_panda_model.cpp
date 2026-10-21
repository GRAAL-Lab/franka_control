#include "fer_model/panda_model.hpp"
#include <gtest/gtest.h>

TEST(PandaModel, HomePoseAndJointLimits)
{
    auto kinematics = fer::PandaModel().ComputeKinematics(fer::HomeJointPosition());
    EXPECT_NEAR(kinematics.toolFrame.translation().x(), 0.3068905666, 1e-8);
    EXPECT_NEAR(kinematics.toolFrame.translation().y(), 0.0, 1e-8);
    EXPECT_NEAR(kinematics.toolFrame.translation().z(), 0.4868820523, 1e-8);
    EXPECT_TRUE(fer::JointsWithinLimits(fer::HomeJointPosition(), 0.05));
    EXPECT_FALSE(fer::JointsWithinLimits(fer::JointVector::Zero()));
}

TEST(PandaModel, JacobianAgreesWithFiniteDifferences)
{
    fer::PandaModel model;
    auto initial = model.ComputeKinematics(fer::HomeJointPosition());
    for (int joint = 0; joint < 7; ++joint) {
        fer::JointVector position = fer::HomeJointPosition();
        position[joint] += 1e-7;
        auto moved = model.ComputeKinematics(position);
        fer::CartesianVector derivative = fer::CartesianError(initial.toolFrame, moved.toolFrame) / 1e-7;
        EXPECT_LT((derivative - initial.toolJacobian.col(joint)).norm(), 1e-6);
    }
}

TEST(PandaModel, RMLModelMatchesSimulatorKinematicsAndToolTransform)
{
    fer::FrankaArmModel arm;
    fer::PandaModel model;
    for (int sample = 0; sample < 4; ++sample) {
        fer::JointVector position = fer::HomeJointPosition();
        position[0] += 0.2 * sample;
        position[2] -= 0.1 * sample;
        arm.JointsPosition(position);
        auto kinematics = model.ComputeKinematics(position);
        EXPECT_LT((arm.TransformationMatrix("Panda_Tool") - kinematics.toolFrame.matrix()).norm(), 1e-10);
        EXPECT_LT((arm.Jacobian("Panda_Tool") - kinematics.toolJacobian).norm(), 1e-10);
    }
    fer::Pose changedTool = fer::MakePose({ 0.01, 0.02, 0.2 }, { 0.1, -0.2, 0.3 });
    arm.SetToolFrame(changedTool);
    EXPECT_TRUE(arm.Jacobian("Panda_Tool").allFinite());
}
