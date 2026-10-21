#include "fer_tcp_bridge/arm_command_parser.hpp"
#include <gtest/gtest.h>
#include <cstring>
#include <limits>

namespace {

TEST(ArmCommandParserTest, DecodesSevenJointPositions)
{
    ArmProtocol::ArmCommandMessage message {};
    message.header = { 0, ArmProtocol::JOINTS_MOVE_ABS, 7 * sizeof(double) };
    std::array<double, 7> positions { 0.1, -0.5, 0.2, -2.0, 0.3, 1.5, 0.7 };
    std::memcpy(
        reinterpret_cast<char*>(&message) + ArmProtocol::HEADER_SIZE, positions.data(), sizeof(positions));
    fer_msgs::srv::ControlCommand::Request request;
    ASSERT_TRUE(fer::ArmCommandParser::ParseCommand(message, request));
    EXPECT_EQ(request.command_type, "move_joints_pos");
    EXPECT_EQ(request.move_type, request.ABSMOVE);
    EXPECT_EQ(request.joint_setpoint, std::vector<double>(positions.begin(), positions.end()));
    message.header.length = 6 * sizeof(double);
    EXPECT_FALSE(fer::ArmCommandParser::ParseCommand(message, request));
}

TEST(ArmCommandParserTest, DecodesSingleJointAndRejectsInvalidIndex)
{
    ArmProtocol::ArmCommandMessage message {};
    message.header
        = { 0, ArmProtocol::JOINT_MOVE_REL, sizeof(ArmProtocol::JointMoveRel) - ArmProtocol::HEADER_SIZE };
    message.jointMoveRel.id = 6;
    message.jointMoveRel.delta_qi = 0.1;
    fer_msgs::srv::ControlCommand::Request request;
    ASSERT_TRUE(fer::ArmCommandParser::ParseCommand(message, request));
    EXPECT_EQ(request.command_type, "move_joint_i_pos");
    EXPECT_EQ(request.move_type, request.RELMOVE);
    EXPECT_EQ(request.joint_index, 6);
    EXPECT_DOUBLE_EQ(request.joint_setpoint[0], 0.1);
    message.jointMoveRel.id = 7;
    EXPECT_FALSE(fer::ArmCommandParser::ParseCommand(message, request));
    message.jointMoveRel.id = -1;
    EXPECT_FALSE(fer::ArmCommandParser::ParseCommand(message, request));
}

TEST(ArmCommandParserTest, DecodesRelativeAxisAndValidatesFrame)
{
    ArmProtocol::ArmCommandMessage message {};
    message.header = { 0, ArmProtocol::EFFECTOR_AXIS_MOVE_REL,
        sizeof(ArmProtocol::EffectorAxisMoveRel) - ArmProtocol::HEADER_SIZE };
    message.effectorAxisMoveRel.axis_id = ArmProtocol::YAW_AXIS;
    message.effectorAxisMoveRel.frame = ArmProtocol::EE;
    message.effectorAxisMoveRel.delta_axis = 0.05;
    fer_msgs::srv::ControlCommand::Request request;
    ASSERT_TRUE(fer::ArmCommandParser::ParseCommand(message, request));
    EXPECT_EQ(request.command_type, "move_cartesian");
    EXPECT_EQ(request.frame_type, request.EE);
    EXPECT_DOUBLE_EQ(request.target_orientation[2], 0.05);
    EXPECT_DOUBLE_EQ(request.target_position[0], 0.0);
    message.effectorAxisMoveRel.axis_id = ArmProtocol::MAX_AXIS_ID;
    EXPECT_FALSE(fer::ArmCommandParser::ParseCommand(message, request));
    message.effectorAxisMoveRel.axis_id = 0;
    message.effectorAxisMoveRel.frame = ArmProtocol::LOCAL;
    EXPECT_FALSE(fer::ArmCommandParser::ParseCommand(message, request));
}

TEST(ArmCommandParserTest, ConvertsGripperPercentageToWidth)
{
    ArmProtocol::ArmCommandMessage message {};
    message.header = { 0, ArmProtocol::GRIPPER_MOVE, sizeof(double) };
    message.gripperMove.percentage = 50.0;
    fer_msgs::srv::ControlCommand::Request request;
    ASSERT_TRUE(fer::ArmCommandParser::ParseCommand(message, request));
    EXPECT_EQ(request.command_type, "gripper_move");
    EXPECT_DOUBLE_EQ(request.gripper_setpoint, 0.04);
    message.gripperMove.percentage = 101.0;
    EXPECT_FALSE(fer::ArmCommandParser::ParseCommand(message, request));
}

TEST(ArmCommandParserTest, RejectsMalformedAndUnsupportedCommands)
{
    ArmProtocol::ArmCommandMessage message {};
    fer_msgs::srv::ControlCommand::Request request;
    message.header = { 0, ArmProtocol::ARM_ENABLE, 0 };
    ASSERT_TRUE(fer::ArmCommandParser::ParseCommand(message, request));
    EXPECT_EQ(request.command_type, "enable");
    message.header.length = 1;
    EXPECT_FALSE(fer::ArmCommandParser::ParseCommand(message, request));
    message.header = { 0, ArmProtocol::ARM_HOMING, 0 };
    EXPECT_FALSE(fer::ArmCommandParser::ParseCommand(message, request));
    message.header = { 0, 999, 0 };
    EXPECT_FALSE(fer::ArmCommandParser::ParseCommand(message, request));
    message.header
        = { 0, ArmProtocol::JOINT_MOVE_VEL, sizeof(ArmProtocol::JointMoveVel) - ArmProtocol::HEADER_SIZE };
    message.jointMoveVel.velocity = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(fer::ArmCommandParser::ParseCommand(message, request));
}

}
