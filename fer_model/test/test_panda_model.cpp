#include <gtest/gtest.h>
#include "fer_model/panda_model.hpp"

TEST(PandaModel, HomePoseAndJointLimits) {
  const auto k = fer::PandaModel().evaluate(fer::kHome);
  EXPECT_NEAR(k.tool.translation().x(), .3068905666, 1e-8);
  EXPECT_NEAR(k.tool.translation().y(), 0., 1e-8);
  EXPECT_NEAR(k.tool.translation().z(), .4868820523, 1e-8);
  EXPECT_TRUE(fer::within_limits(fer::kHome, .05));
  EXPECT_FALSE(fer::within_limits(fer::Joints::Zero()));
}

TEST(PandaModel, JacobianAgreesWithFiniteDifferences) {
  const fer::PandaModel model;
  const auto initial = model.evaluate(fer::kHome);
  for (int i = 0; i < 7; ++i) {
    fer::Joints q = fer::kHome;
    q[i] += 1e-7;
    const auto moved = model.evaluate(q);
    const auto derivative = fer::pose_error(initial.tool, moved.tool) / 1e-7;
    EXPECT_LT((derivative - initial.jacobian.col(i)).norm(), 1e-6);
  }
}
