#include "fer_ctrl/task_priority_controller.hpp"
#include <gtest/gtest.h>
#include <filesystem>
#include <limits>
#include <unistd.h>

namespace {

class ControllerTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        configuration_.readFile(FER_KCL_CONFIG);
        configuration_.lookup("actionTransitionDuration") = 0.0;
        controller_ = MakeController();
        feedback_.connected = true;
        feedback_.ready = true;
        ASSERT_TRUE(controller_->SetFeedback(feedback_, currentTime_));
    }

    void TearDown() override
    {
        for (const auto& file : configurationFiles_) {
            std::filesystem::remove(file);
        }
    }

    std::unique_ptr<fer::TaskPriorityController> MakeController(bool simulatedDriver = false)
    {
        std::string file = "/tmp/fer_kcl_test_" + std::to_string(getpid()) + "_"
            + std::to_string(configurationFiles_.size()) + ".conf";
        configuration_.writeFile(file.c_str());
        configurationFiles_.push_back(file);
        return std::make_unique<fer::TaskPriorityController>(file, simulatedDriver);
    }

    fer::CommandResult SendCommand(const std::string& command)
    {
        fer::CommandRequest request;
        request.commandType = command;
        return controller_->ExecuteCommand(request, currentTime_);
    }

    void Enable()
    {
        ASSERT_TRUE(SendCommand("enable").success);
        EXPECT_EQ(controller_->GetState(), "disabled");
        controller_->Run(currentTime_);
        EXPECT_EQ(controller_->GetState(), "enabling");
        feedback_.controlActive = true;
        controller_->SetFeedback(feedback_, currentTime_);
        controller_->Run(currentTime_);
        EXPECT_EQ(controller_->GetState(), "enabling");
        controller_->Run(currentTime_);
        ASSERT_EQ(controller_->GetState(), "idle");
    }

    std::unique_ptr<fer::TaskPriorityController> controller_;
    libconfig::Config configuration_;
    std::vector<std::string> configurationFiles_;
    fer::RobotFeedback feedback_;
    double currentTime_ = 1.0;
};

TEST_F(ControllerTest, LoadsRealTaskPriorityArchitecture)
{
    EXPECT_EQ(controller_->GetTasksMap().size(), 7u);
    auto manager = controller_->GetActionManager();
    EXPECT_EQ(manager->GetHierarchy().size(), 5u);
    EXPECT_EQ(manager->CurrentActionID(), "Idle");
    EXPECT_EQ(manager->GetAction("MoveTool")->PriorityLevels().front()->ID(), "PL_Safety");
    EXPECT_EQ(controller_->GetConfiguration().saturationMax.size(), 7);
}

TEST_F(ControllerTest, RejectsUnknownTaskAndPriorityLevel)
{
    configuration_.lookup("priorityLevels")[0]["tasks"][0] = "UnknownTask";
    EXPECT_THROW(MakeController(), std::invalid_argument);
    configuration_.lookup("priorityLevels")[0]["tasks"][0] = "JointsLimit";
    configuration_.lookup("actions")[0]["levels"][0] = "UnknownLevel";
    EXPECT_THROW(MakeController(), std::invalid_argument);
}

TEST_F(ControllerTest, RejectsDuplicatePrioritiesAndInvalidJointDimensions)
{
    configuration_.lookup("priorityLevels")[1]["name"] = "PL_Safety";
    EXPECT_THROW(MakeController(), std::invalid_argument);
    configuration_.lookup("priorityLevels")[1]["name"] = "PL_MoveJointsPosition";
    configuration_.lookup("tasks.JointsLimit.activationStartDistanceGreaterThanParams").remove(6);
    EXPECT_THROW(MakeController(), std::invalid_argument);
}

TEST_F(ControllerTest, RequiresExplicitEnableAndDriverAcknowledgement)
{
    EXPECT_FALSE(SendCommand("park").success);
    Enable();
    EXPECT_TRUE(controller_->Run(currentTime_).jointVelocity.isZero());
    EXPECT_TRUE(SendCommand("disable").success);
    EXPECT_EQ(controller_->GetState(), "idle");
    EXPECT_TRUE(controller_->Run(currentTime_).jointVelocity.isZero());
    EXPECT_FALSE(controller_->IsEnabled());
}

TEST_F(ControllerTest, RejectsMalformedGoalsWithoutChangingState)
{
    Enable();
    fer::CommandRequest request;
    request.commandType = "move_joint_i_pos";
    request.jointSetpoint = { 0.1 };
    request.jointIndex = -1;
    EXPECT_FALSE(controller_->ExecuteCommand(request, currentTime_).success);
    request.jointIndex = 7;
    EXPECT_FALSE(controller_->ExecuteCommand(request, currentTime_).success);
    request.jointIndex = 0;
    request.jointSetpoint = { std::numeric_limits<double>::quiet_NaN() };
    EXPECT_FALSE(controller_->ExecuteCommand(request, currentTime_).success);
    request.jointSetpoint = { 100.0 };
    EXPECT_FALSE(controller_->ExecuteCommand(request, currentTime_).success);
    request.jointSetpoint = { 0.1 };
    request.moveType = "invalid";
    EXPECT_FALSE(controller_->ExecuteCommand(request, currentTime_).success);
    EXPECT_EQ(controller_->GetState(), "idle");
}

TEST_F(ControllerTest, ConfiguredTaskGainChangesSolverOutput)
{
    Enable();
    fer::CommandRequest request;
    request.commandType = "move_joint_i_pos";
    request.jointSetpoint = { 0.1 };
    ASSERT_TRUE(controller_->ExecuteCommand(request, currentTime_).success);
    double originalVelocity = controller_->Run(currentTime_).jointVelocity[0];
    EXPECT_GT(originalVelocity, 0.05);

    configuration_.lookup("tasks.JointsPosition.gain") = 0.1;
    controller_ = MakeController();
    feedback_.controlActive = false;
    controller_->SetFeedback(feedback_, currentTime_);
    Enable();
    ASSERT_TRUE(controller_->ExecuteCommand(request, currentTime_).success);
    double reducedVelocity = controller_->Run(currentTime_).jointVelocity[0];
    EXPECT_GT(reducedVelocity, 0.0);
    EXPECT_LT(reducedVelocity, originalVelocity * 0.2);
}

TEST_F(ControllerTest, ConfiguredTaskEnableChangesSolverOutput)
{
    configuration_.lookup("tasks.JointsPosition.enable") = false;
    controller_ = MakeController();
    controller_->SetFeedback(feedback_, currentTime_);
    Enable();
    fer::CommandRequest request;
    request.commandType = "move_joint_i_pos";
    request.jointSetpoint = { 0.1 };
    ASSERT_TRUE(controller_->ExecuteCommand(request, currentTime_).success);
    EXPECT_TRUE(controller_->Run(currentTime_).jointVelocity.isZero(1e-8));
}

TEST_F(ControllerTest, JointLimitTaskHonorsItsEnableFlag)
{
    configuration_.lookup("tasks.JointsLimit.enable") = false;
    controller_ = MakeController();
    EXPECT_FALSE(controller_->GetTasksMap().at("JointsLimit").task->Enabled());
}

TEST_F(ControllerTest, SafetyPlanePushesTowardConfiguredSafeSide)
{
    for (double signedDistance : { 0.03, 0.0, -0.02 }) {
        double toolHeight = feedback_.toolFrame.translation().z();
        configuration_.lookup("obstacles")[0]["offset"] = signedDistance - toolHeight;
        controller_ = MakeController();
        feedback_.controlActive = false;
        controller_->SetFeedback(feedback_, currentTime_);
        Enable();
        fer::CommandRequest request;
        request.commandType = "move_joint_i_vel";
        request.jointSetpoint = { 0.0 };
        ASSERT_TRUE(controller_->ExecuteCommand(request, currentTime_).success);
        fer::ControlOutput output = controller_->Run(currentTime_);
        ASSERT_TRUE(output.jointVelocity.allFinite());
        EXPECT_GT((feedback_.toolJacobian * output.jointVelocity)[2], 0.0);
    }
}

TEST_F(ControllerTest, ConfiguredPriorityOrderResolvesConflictingTasks)
{
    // Activate posture and position together, with posture at the higher priority.
    configuration_.lookup("priorityLevels")[1]["tasks"][0] = "JointsPosture";
    configuration_.lookup("priorityLevels")[4]["tasks"][0] = "JointsPosition";
    configuration_.lookup("actions")[2]["levels"].add(libconfig::Setting::TypeString) = "PL_Posture";
    controller_ = MakeController();
    controller_->SetFeedback(feedback_, currentTime_);
    Enable();
    fer::CommandRequest request;
    request.commandType = "move_joint_i_pos";
    request.jointSetpoint = { 0.1 };
    ASSERT_TRUE(controller_->ExecuteCommand(request, currentTime_).success);
    EXPECT_LT(controller_->Run(currentTime_).jointVelocity.norm(), 1e-5);
}

TEST_F(ControllerTest, StaleFeedbackStopsAndDoesNotRestart)
{
    Enable();
    ASSERT_TRUE(SendCommand("park").success);
    EXPECT_GT(controller_->Run(currentTime_).jointVelocity.norm(), 0.0);
    currentTime_ += 0.3;
    EXPECT_TRUE(controller_->Run(currentTime_).jointVelocity.isZero());
    EXPECT_EQ(controller_->GetState(), "disabled");
    controller_->SetFeedback(feedback_, currentTime_);
    EXPECT_TRUE(controller_->Run(currentTime_).jointVelocity.isZero());
    EXPECT_TRUE(SendCommand("stop").success);
}

TEST_F(ControllerTest, VelocityCommandsExpire)
{
    Enable();
    fer::CommandRequest request;
    request.commandType = "move_joint_i_vel";
    request.jointSetpoint = { 0.1 };
    ASSERT_TRUE(controller_->ExecuteCommand(request, currentTime_).success);
    EXPECT_NEAR(controller_->Run(currentTime_).jointVelocity[0], 0.1, 1e-4);
    currentTime_ += 0.31;
    controller_->SetFeedback(feedback_, currentTime_);
    EXPECT_TRUE(controller_->Run(currentTime_).jointVelocity.isZero());
    EXPECT_EQ(controller_->GetState(), "move_joints_vel");
    controller_->Run(currentTime_);
    EXPECT_EQ(controller_->GetState(), "idle");
}

TEST_F(ControllerTest, FaultNeedsRecoveryAndExplicitEnable)
{
    Enable();
    feedback_.hasError = true;
    controller_->SetFeedback(feedback_, currentTime_);
    EXPECT_TRUE(controller_->Run(currentTime_).jointVelocity.isZero());
    EXPECT_EQ(controller_->GetState(), "fault");
    EXPECT_FALSE(SendCommand("enable").success);
    feedback_.controlActive = false;
    controller_->SetFeedback(feedback_, currentTime_);
    ASSERT_TRUE(SendCommand("recover").success);
    controller_->Run(currentTime_);
    EXPECT_FALSE(SendCommand("enable").success);
    feedback_.hasError = false;
    controller_->SetFeedback(feedback_, currentTime_);
    EXPECT_EQ(controller_->GetState(), "disabled");
    EXPECT_TRUE(SendCommand("enable").success);
}

TEST_F(ControllerTest, EnableTimesOutAndStopCancelsPendingEnable)
{
    ASSERT_TRUE(SendCommand("enable").success);
    ASSERT_TRUE(SendCommand("stop").success);
    controller_->Run(currentTime_);
    EXPECT_FALSE(controller_->IsEnabled());
    ASSERT_TRUE(SendCommand("enable").success);
    controller_->Run(currentTime_);
    currentTime_ += 5.1;
    controller_->SetFeedback(feedback_, currentTime_);
    EXPECT_TRUE(controller_->Run(currentTime_).jointVelocity.isZero());
    EXPECT_EQ(controller_->GetState(), "enabling");
    controller_->Run(currentTime_);
    EXPECT_EQ(controller_->GetState(), "fault");
}

TEST_F(ControllerTest, ParksAndRequiresUnpark)
{
    Enable();
    ASSERT_TRUE(SendCommand("park").success);
    feedback_.jointPosition = controller_->GetConfiguration().parkedJointsPos;
    controller_->SetFeedback(feedback_, currentTime_);
    EXPECT_TRUE(controller_->Run(currentTime_).jointVelocity.isZero());
    EXPECT_EQ(controller_->GetState(), "parking");
    controller_->Run(currentTime_);
    EXPECT_EQ(controller_->GetState(), "parked");
    EXPECT_TRUE(SendCommand("stop").success);
    EXPECT_EQ(controller_->GetState(), "parked");
    EXPECT_FALSE(SendCommand("move_joints_pos").success);
    EXPECT_TRUE(SendCommand("unpark").success);
}

TEST_F(ControllerTest, RelativeCartesianFramesHaveDefinedSemantics)
{
    Enable();
    fer::CommandRequest request;
    request.commandType = "move_cartesian";
    request.moveType = "relative";
    request.frameType = 0;
    request.targetPosition = { 0.0, 0.0, 0.01 };
    ASSERT_TRUE(controller_->ExecuteCommand(request, currentTime_).success);
    fer::Pose expected = feedback_.toolFrame * fer::MakePose(request.targetPosition, Eigen::Vector3d::Zero());
    EXPECT_LT((controller_->GetRobotInfo().cartesianGoalFrame.matrix() - expected.matrix()).norm(), 1e-12);
    request.frameType = 1;
    ASSERT_TRUE(controller_->ExecuteCommand(request, currentTime_).success);
    EXPECT_LT((controller_->GetRobotInfo().cartesianGoalFrame.translation()
                  - feedback_.toolFrame.translation() - request.targetPosition)
                  .norm(),
        1e-12);
}

TEST_F(ControllerTest, CartesianLoopConvergesThroughRealTPIKSolver)
{
    Enable();
    fer::CommandRequest request;
    request.commandType = "move_cartesian";
    request.moveType = "relative";
    request.targetPosition = { 0.02, 0.01, 0.01 };
    ASSERT_TRUE(controller_->ExecuteCommand(request, currentTime_).success);
    controller_->Run(currentTime_);
    for (int step = 0; step < 2000 && controller_->GetState() != "idle"; ++step) {
        fer::ControlOutput output = controller_->Run(currentTime_);
        ASSERT_TRUE(output.jointVelocity.allFinite());
        EXPECT_LE(output.jointVelocity.cwiseAbs().maxCoeff(), 0.350001);
        feedback_.jointVelocity = output.jointVelocity;
        feedback_.jointPosition += output.jointVelocity * 0.01;
        auto kinematics = fer::PandaModel().ComputeKinematics(feedback_.jointPosition);
        feedback_.toolFrame = kinematics.toolFrame;
        feedback_.toolJacobian = kinematics.toolJacobian;
        currentTime_ += 0.01;
        controller_->SetFeedback(feedback_, currentTime_);
    }
    EXPECT_EQ(controller_->GetState(), "idle");
    EXPECT_LT(fer::CartesianError(feedback_.toolFrame, controller_->GetRobotInfo().cartesianGoalFrame)
                  .head<3>()
                  .norm(),
        0.0021);
}

TEST_F(ControllerTest, InvalidFeedbackFaultsImmediately)
{
    Enable();
    feedback_.jointPosition[0] = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(controller_->SetFeedback(feedback_, currentTime_));
    EXPECT_EQ(controller_->GetState(), "fault");
    EXPECT_TRUE(controller_->Run(currentTime_).jointVelocity.isZero());
}

TEST_F(ControllerTest, CommandsWaitForTheControlTimer)
{
    Enable();
    fer::CommandRequest request;
    request.commandType = "move_joint_i_pos";
    request.jointSetpoint = { 0.1 };
    ASSERT_TRUE(controller_->ExecuteCommand(request, currentTime_).success);
    EXPECT_EQ(controller_->GetState(), "idle");
    EXPECT_EQ(controller_->GetActionManager()->CurrentActionID(), "Idle");

    EXPECT_GT(controller_->Run(currentTime_).jointVelocity[0], 0.0);
    EXPECT_EQ(controller_->GetState(), "move_joints_pos");
    EXPECT_EQ(controller_->GetActionManager()->CurrentActionID(), "MoveJointsPosition");

    ASSERT_TRUE(SendCommand("stop").success);
    EXPECT_EQ(controller_->GetState(), "move_joints_pos");
    EXPECT_TRUE(controller_->Run(currentTime_).jointVelocity.isZero());
    EXPECT_EQ(controller_->GetState(), "idle");
}

TEST_F(ControllerTest, StateCompletionWaitsForTheNextControlCycle)
{
    Enable();
    fer::CommandRequest request;
    request.commandType = "move_joint_i_pos";
    request.jointSetpoint = { 0.1 };
    ASSERT_TRUE(controller_->ExecuteCommand(request, currentTime_).success);
    controller_->Run(currentTime_);

    feedback_.jointPosition[0] = 0.1;
    controller_->SetFeedback(feedback_, currentTime_);
    controller_->Run(currentTime_);
    EXPECT_EQ(controller_->GetState(), "move_joints_pos");
    EXPECT_EQ(controller_->GetActionManager()->CurrentActionID(), "MoveJointsPosition");

    EXPECT_TRUE(controller_->Run(currentTime_).jointVelocity.isZero());
    EXPECT_EQ(controller_->GetState(), "idle");
    EXPECT_EQ(controller_->GetActionManager()->CurrentActionID(), "Idle");
}

TEST_F(ControllerTest, RejectsMotionCommandsWhileParkingAndUnparking)
{
    Enable();
    ASSERT_TRUE(SendCommand("park").success);
    controller_->Run(currentTime_);
    ASSERT_EQ(controller_->GetState(), "parking");

    for (const std::string state : { "parking", "unparking" }) {
        SCOPED_TRACE(state);
        ASSERT_EQ(controller_->GetState(), state);
        for (const std::string command : { "enable", "park", "unpark", "move_joints_pos", "move_joint_i_pos",
                 "move_joints_vel", "move_joint_i_vel", "move_cartesian" }) {
            SCOPED_TRACE(command);
            fer::CommandRequest request;
            request.commandType = command;
            request.jointSetpoint.assign(
                feedback_.jointPosition.data(), feedback_.jointPosition.data() + fer::ARM_JOINTS);
            if (command == "move_joints_vel") {
                request.jointSetpoint.assign(fer::ARM_JOINTS, 0.1);
            } else if (command == "move_joint_i_pos" || command == "move_joint_i_vel") {
                request.jointSetpoint = { 0.1 };
            }
            auto result = controller_->ExecuteCommand(request, currentTime_);
            EXPECT_FALSE(result.success);
            EXPECT_NE(result.message.find("not allowed in state " + state), std::string::npos);
            EXPECT_EQ(controller_->GetState(), state);
        }

        if (state == "parking") {
            feedback_.jointPosition = controller_->GetConfiguration().parkedJointsPos;
            controller_->SetFeedback(feedback_, currentTime_);
            controller_->Run(currentTime_);
            controller_->Run(currentTime_);
            ASSERT_EQ(controller_->GetState(), "parked");
            ASSERT_TRUE(SendCommand("unpark").success);
            controller_->Run(currentTime_);
        }
    }
}

TEST_F(ControllerTest, IdleAllowsUnpark)
{
    Enable();
    ASSERT_TRUE(SendCommand("unpark").success);
    EXPECT_EQ(controller_->GetState(), "idle");
    controller_->Run(currentTime_);
    EXPECT_EQ(controller_->GetState(), "unparking");
}

TEST_F(ControllerTest, EnableInIdleKeepsTheControllerEnabled)
{
    Enable();
    ASSERT_TRUE(SendCommand("enable").success);
    EXPECT_TRUE(controller_->Run(currentTime_).jointVelocity.isZero());
    EXPECT_EQ(controller_->GetState(), "idle");
    EXPECT_TRUE(controller_->IsEnabled());
}

TEST_F(ControllerTest, SimulationAllowsJointPositionMovesFromParked)
{
    controller_ = MakeController(true);
    controller_->SetFeedback(feedback_, currentTime_);
    Enable();
    ASSERT_TRUE(SendCommand("park").success);
    controller_->Run(currentTime_);
    feedback_.jointPosition = controller_->GetConfiguration().parkedJointsPos;
    controller_->SetFeedback(feedback_, currentTime_);
    controller_->Run(currentTime_);
    controller_->Run(currentTime_);
    ASSERT_EQ(controller_->GetState(), "parked");

    fer::CommandRequest request;
    request.commandType = "move_joints_pos";
    request.jointSetpoint.assign(
        fer::HomeJointPosition().data(), fer::HomeJointPosition().data() + fer::ARM_JOINTS);
    ASSERT_TRUE(controller_->ExecuteCommand(request, currentTime_).success);
    EXPECT_EQ(controller_->GetState(), "parked");
    EXPECT_GT(controller_->Run(currentTime_).jointVelocity.norm(), 0.0);
    EXPECT_EQ(controller_->GetState(), "move_joints_pos");
}

TEST_F(ControllerTest, StopAndDisableCancelCommandsBeforeTheTimerAppliesThem)
{
    for (const std::string command : { "stop", "disable" }) {
        SCOPED_TRACE(command);
        controller_ = MakeController();
        feedback_.controlActive = false;
        controller_->SetFeedback(feedback_, currentTime_);
        ASSERT_TRUE(SendCommand("enable").success);
        ASSERT_TRUE(SendCommand(command).success);
        EXPECT_TRUE(controller_->Run(currentTime_).jointVelocity.isZero());
        EXPECT_EQ(controller_->GetState(), "disabled");

        Enable();
        ASSERT_TRUE(SendCommand("park").success);
        ASSERT_TRUE(SendCommand(command).success);
        EXPECT_TRUE(controller_->Run(currentTime_).jointVelocity.isZero());
        EXPECT_EQ(controller_->GetState(), command == "stop" ? "idle" : "disabled");
    }
}

TEST_F(ControllerTest, RepeatedVelocityCommandsDoNotReenterTheState)
{
    Enable();
    fer::CommandRequest request;
    request.commandType = "move_joint_i_vel";
    request.jointSetpoint = { 0.1 };
    ASSERT_TRUE(controller_->ExecuteCommand(request, currentTime_).success);
    controller_->Run(currentTime_);
    double stateEntryTime = controller_->GetRobotInfo().stateEntryTime;

    currentTime_ += 0.01;
    controller_->SetFeedback(feedback_, currentTime_);
    request.jointSetpoint = { 0.2 };
    ASSERT_TRUE(controller_->ExecuteCommand(request, currentTime_).success);
    EXPECT_NEAR(controller_->Run(currentTime_).jointVelocity[0], 0.2, 1e-4);
    EXPECT_EQ(controller_->GetState(), "move_joints_vel");
    EXPECT_EQ(controller_->GetRobotInfo().stateEntryTime, stateEntryTime);
}

}
