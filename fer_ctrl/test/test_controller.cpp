#include <limits>
#include <gtest/gtest.h>
#include "fer_ctrl/controller.hpp"

namespace {
class ControllerTest : public ::testing::Test {
 protected:
  fer::Controller controller;
  fer::Feedback f;
  double time{1.};
  void SetUp() override {
    f.connected = f.ready = true;
    const auto model = fer::PandaModel().evaluate(f.q);
    f.tool = model.tool; f.jacobian = model.jacobian;
    ASSERT_TRUE(controller.feedback(f, time));
  }
  fer::Result send(fer::commands::Type type) {
    fer::commands::Command c; c.type = type;
    return controller.execute(c, time);
  }
  void enable() {
    ASSERT_TRUE(send(fer::commands::Type::Enable).success);
    EXPECT_EQ(controller.state(), fer::states::State::Enabling);
    f.active = true;
    controller.feedback(f, time);
    controller.update(time);
    EXPECT_EQ(controller.state(), fer::states::State::Idle);
  }
};

TEST_F(ControllerTest, RequiresExplicitEnableAndDriverAcknowledgement) {
  EXPECT_FALSE(send(fer::commands::Type::Park).success);
  enable();
  EXPECT_TRUE(controller.update(time).velocity.isZero());
  EXPECT_TRUE(send(fer::commands::Type::Disable).success);
  EXPECT_FALSE(controller.enabled());
}
TEST_F(ControllerTest, RejectsMalformedCommandsWithoutChangingState) {
  enable();
  fer::commands::Command c; c.type = fer::commands::Type::SingleJointPosition;
  c.joints = {.1}; c.joint_index = -1;
  EXPECT_FALSE(controller.execute(c, time).success);
  c.joint_index = 7; EXPECT_FALSE(controller.execute(c, time).success);
  c.joint_index = 0; c.joints = {std::numeric_limits<double>::quiet_NaN()};
  EXPECT_FALSE(controller.execute(c, time).success);
  c.joints = {100.}; EXPECT_FALSE(controller.execute(c, time).success);
  c.joints = {.1}; c.valid_move_type = false;
  EXPECT_FALSE(controller.execute(c, time).success);
  EXPECT_EQ(controller.state(), fer::states::State::Idle);
}
TEST_F(ControllerTest, RelativeSingleJointTargetAndVelocityBounds) {
  enable();
  fer::commands::Command c; c.type = fer::commands::Type::SingleJointPosition;
  c.joint_index = 1; c.relative = true; c.joints = {.1};
  ASSERT_TRUE(controller.execute(c, time).success);
  const auto output = controller.update(time);
  EXPECT_NEAR(output.velocity[1], .15, 1e-8);
  EXPECT_LE(output.velocity.cwiseAbs().maxCoeff(), .35);
}
TEST_F(ControllerTest, StaleFeedbackStopsAndDoesNotRestart) {
  enable();
  ASSERT_TRUE(send(fer::commands::Type::Park).success);
  EXPECT_FALSE(controller.update(time).velocity.isZero());
  time += .3;
  EXPECT_TRUE(controller.update(time).velocity.isZero());
  EXPECT_EQ(controller.state(), fer::states::State::Disabled);
  controller.feedback(f, time);
  EXPECT_TRUE(controller.update(time).velocity.isZero());
  EXPECT_FALSE(controller.enabled());
  EXPECT_TRUE(send(fer::commands::Type::Stop).success);
}
TEST_F(ControllerTest, VelocityCommandsExpire) {
  enable();
  fer::commands::Command c; c.type = fer::commands::Type::SingleJointVelocity;
  c.joint_index = 0; c.joints = {.1};
  ASSERT_TRUE(controller.execute(c, time).success);
  EXPECT_NEAR(controller.update(time).velocity[0], .1, 1e-12);
  time += .31; controller.feedback(f, time);
  EXPECT_TRUE(controller.update(time).velocity.isZero());
  EXPECT_EQ(controller.state(), fer::states::State::Idle);
}
TEST_F(ControllerTest, DriverFaultNeedsRecoveryAndExplicitEnable) {
  enable();
  f.error = true; controller.feedback(f, time);
  EXPECT_TRUE(controller.update(time).velocity.isZero());
  EXPECT_EQ(controller.state(), fer::states::State::Fault);
  EXPECT_FALSE(send(fer::commands::Type::Enable).success);
  ASSERT_TRUE(send(fer::commands::Type::Recover).success);
  EXPECT_FALSE(send(fer::commands::Type::Enable).success);
  f.error = false; f.active = false; controller.feedback(f, time);
  EXPECT_EQ(controller.state(), fer::states::State::Disabled);
  EXPECT_TRUE(send(fer::commands::Type::Enable).success);
}
TEST_F(ControllerTest, EnableTimesOut) {
  ASSERT_TRUE(send(fer::commands::Type::Enable).success);
  time += 5.1; controller.feedback(f, time);
  EXPECT_TRUE(controller.update(time).velocity.isZero());
  EXPECT_EQ(controller.state(), fer::states::State::Fault);
}
TEST_F(ControllerTest, ParksAndRequiresUnpark) {
  enable(); ASSERT_TRUE(send(fer::commands::Type::Park).success);
  f.q = fer::Configuration().parked; controller.feedback(f, time);
  EXPECT_TRUE(controller.update(time).velocity.isZero());
  EXPECT_EQ(controller.state(), fer::states::State::Parked);
  EXPECT_TRUE(send(fer::commands::Type::Stop).success);
  EXPECT_EQ(controller.state(), fer::states::State::Parked);
  fer::commands::Command c; c.type = fer::commands::Type::SingleJointVelocity; c.joints = {.1};
  EXPECT_FALSE(controller.execute(c, time).success);
  EXPECT_TRUE(send(fer::commands::Type::Unpark).success);
}
TEST_F(ControllerTest, StopCancelsPendingEnable) {
  ASSERT_TRUE(send(fer::commands::Type::Enable).success);
  ASSERT_TRUE(send(fer::commands::Type::Stop).success);
  EXPECT_FALSE(controller.enabled());
  f.active = true; controller.feedback(f, time);
  EXPECT_TRUE(controller.update(time).velocity.isZero());
  EXPECT_EQ(controller.state(), fer::states::State::Disabled);
}
TEST_F(ControllerTest, JointLimitDamperBlocksOutwardMotionAndAllowsRetreat) {
  enable(); f.q[0] = fer::kUpper[0] - .05; controller.feedback(f, time);
  fer::commands::Command c; c.type = fer::commands::Type::SingleJointVelocity;
  c.joints = {.1};
  ASSERT_TRUE(controller.execute(c, time).success);
  const auto output = controller.update(time);
  EXPECT_NEAR(output.velocity[0], 0., 1e-12);
  EXPECT_TRUE(output.joint_limit_active);
  c.joints = {-.1}; ASSERT_TRUE(controller.execute(c, time).success);
  EXPECT_NEAR(controller.update(time).velocity[0], -.1, 1e-12);
}
TEST_F(ControllerTest, RelativeCartesianFramesHaveDefinedSemantics) {
  enable();
  fer::commands::Command c; c.type = fer::commands::Type::Cartesian;
  c.relative = true; c.xyz = {0., 0., .01}; c.frame = 0;
  ASSERT_TRUE(controller.execute(c, time).success);
  EXPECT_LT((controller.target().translation() - f.tool.translation() - f.tool.linear() * c.xyz).norm(), 1e-12);
  c.frame = 1; ASSERT_TRUE(controller.execute(c, time).success);
  EXPECT_LT((controller.target().translation() - f.tool.translation() - c.xyz).norm(), 1e-12);
  c.relative = false; c.frame = 0;
  EXPECT_FALSE(controller.execute(c, time).success);
}
TEST_F(ControllerTest, CartesianLoopConvergesWithSevenJointModel) {
  enable();
  fer::commands::Command c; c.type = fer::commands::Type::Cartesian;
  c.relative = true; c.frame = 1; c.xyz = {.02, .01, .01};
  ASSERT_TRUE(controller.execute(c, time).success);
  for (int i = 0; i < 1500 && controller.state() != fer::states::State::Idle; ++i) {
    const auto output = controller.update(time);
    EXPECT_TRUE(output.velocity.allFinite());
    EXPECT_LE(output.velocity.cwiseAbs().maxCoeff(), .350000001);
    f.dq = output.velocity; f.q += f.dq * .01;
    const auto model = fer::PandaModel().evaluate(f.q);
    f.tool = model.tool; f.jacobian = model.jacobian;
    time += .01; controller.feedback(f, time);
  }
  EXPECT_EQ(controller.state(), fer::states::State::Idle);
  EXPECT_LT(fer::pose_error(f.tool, controller.target()).head<3>().norm(), .0021);
}
TEST_F(ControllerTest, InvalidFeedbackImmediatelyFaults) {
  enable(); f.q[0] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(controller.feedback(f, time));
  EXPECT_EQ(controller.state(), fer::states::State::Fault);
  EXPECT_TRUE(controller.update(time).velocity.isZero());
}
TEST(ControllerConfiguration, RejectsInvalidPosturesAndParameters) {
  fer::Configuration config; config.parked.setZero();
  EXPECT_THROW(fer::Controller c(config), std::invalid_argument);
  config = {}; config.damping = 0.;
  EXPECT_THROW(fer::Controller c(config), std::invalid_argument);
}
}  // namespace
