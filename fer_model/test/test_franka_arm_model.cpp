#include "fer_model/franka_arm_model.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>

TEST(FrankaArmModel, HomePoseAndJointLimits)
{
    fer::FrankaArmModel model;
    auto frame = model.TransformationMatrix(model.ToolFrameID());
    EXPECT_NEAR(frame.TranslationVector().x(), 0.3068905666, 1e-8);
    EXPECT_NEAR(frame.TranslationVector().y(), 0.0, 1e-8);
    EXPECT_NEAR(frame.TranslationVector().z(), 0.4868820523, 1e-8);
    EXPECT_EQ(model.NumJoints(), fer::ARM_JOINTS);
    EXPECT_TRUE(fer::JointsWithinLimits(fer::HomeJointPosition(), 0.05));
    EXPECT_FALSE(fer::JointsWithinLimits(fer::JointVector::Zero()));
}

TEST(FrankaArmModel, JacobianAgreesWithFiniteDifferences)
{
    for (double toolOffset : { 0.0, fer::DEFAULT_TOOL_OFFSET, 0.2 }) {
        SCOPED_TRACE(toolOffset);
        fer::FrankaArmModel model("Panda", toolOffset);
        for (int sample = 0; sample < 4; ++sample) {
            fer::JointVector position = fer::HomeJointPosition();
            position[0] += 0.2 * sample;
            position[2] -= 0.1 * sample;
            model.JointsPosition(position);
            const auto initial = model.TransformationMatrix(model.ToolFrameID());
            const auto jacobian = model.Jacobian(model.ToolFrameID());
            for (std::size_t joint = 0; joint < fer::ARM_JOINTS; ++joint) {
                auto movedPosition = position;
                movedPosition[joint] += 1e-7;
                model.JointsPosition(movedPosition);
                const auto moved = model.TransformationMatrix(model.ToolFrameID());
                Eigen::Vector6d derivative = rml::CartesianError(initial, moved) / 1e-7;
                EXPECT_LT((derivative - jacobian.col(joint)).norm(), 1e-6);
            }
        }
    }
}

TEST(FrankaArmModel, ToolFrameUpdatesTransformAndJacobian)
{
    fer::FrankaArmModel model;
    const auto originalFrame = model.TransformationMatrix(model.ToolFrameID());
    const Eigen::TransformationMatrix flangeFrame = originalFrame * model.FlangeToTool().inverse();
    const auto changedTool = fer::MakeTransformation({ 0.01, 0.02, 0.2 }, { 0.1, -0.2, 0.3 });
    model.SetToolFrame(changedTool);
    const auto initial = model.TransformationMatrix(model.ToolFrameID());
    EXPECT_LT((initial - flangeFrame * changedTool).norm(), 1e-10);
    EXPECT_LT((model.FlangeToTool() - changedTool).norm(), 1e-12);

    const auto jacobian = model.Jacobian(model.ToolFrameID());
    for (std::size_t joint = 0; joint < fer::ARM_JOINTS; ++joint) {
        fer::JointVector position = fer::HomeJointPosition();
        position[joint] += 1e-7;
        model.JointsPosition(position);
        const auto moved = model.TransformationMatrix(model.ToolFrameID());
        Eigen::Vector6d derivative = rml::CartesianError(initial, moved) / 1e-7;
        EXPECT_LT((derivative - jacobian.col(joint)).norm(), 1e-6);
    }
}

TEST(FrankaArmModel, ToolOffsetMovesToolAlongFlangeAxis)
{
    fer::FrankaArmModel standard;
    fer::FrankaArmModel extended("Panda", fer::DEFAULT_TOOL_OFFSET + 0.05);
    const auto standardFrame = standard.TransformationMatrix(standard.ToolFrameID());
    const auto extendedFrame = extended.TransformationMatrix(extended.ToolFrameID());
    EXPECT_LT((extendedFrame.TranslationVector() - standardFrame.TranslationVector()
                  - 0.05 * standardFrame.RotationMatrix().col(2))
                  .norm(),
        1e-12);
    EXPECT_LT((extendedFrame.RotationMatrix() - standardFrame.RotationMatrix()).norm(), 1e-12);
    EXPECT_GT((extended.Jacobian(extended.ToolFrameID()) - standard.Jacobian(standard.ToolFrameID())).norm(),
        0.01);
}

TEST(FrankaArmModel, RejectsInvalidToolOffset)
{
    for (double toolOffset : { -0.01, std::numeric_limits<double>::infinity(),
             std::numeric_limits<double>::quiet_NaN() }) {
        EXPECT_THROW((fer::FrankaArmModel{ "Panda", toolOffset }), std::invalid_argument);
    }
}

TEST(FrankaArmModel, CustomArmIDIdentifiesToolAndJointFrames)
{
    fer::FrankaArmModel model("CustomArm");
    EXPECT_EQ(model.ToolFrameID(), "CustomArm_Tool");
    for (std::size_t joint = 0; joint < fer::ARM_JOINTS; ++joint) {
        const auto frame = model.TransformationMatrix(
            model.ID() + rml::FrameID::Joint + std::to_string(joint));
        EXPECT_TRUE(frame.allFinite());
    }
    EXPECT_TRUE(model.TransformationMatrix(model.ToolFrameID()).allFinite());
}
