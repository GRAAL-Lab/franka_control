#include "fer_tcp_bridge/arm_command_parser.hpp"

#include <ctrl_toolbox_internal/Futils.h>
#include "fer_msgs/conversions.hpp"
#include "fer_msgs/fer_defines.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <future>

namespace fer {
namespace {

constexpr std::size_t MAX_PENDING_COMMANDS = 64;
constexpr double COMMAND_RESPONSE_TIMEOUT = 5.0; // Seconds on the owning node's clock.

}

ArmCommandParser::ArmCommandParser(rclcpp::Node* node)
    : node_(node)
{
    controlCommandClient_ = node_->create_client<fer_msgs::srv::ControlCommand>(topicnames::control_command);
}

bool ArmCommandParser::ParseCommand(
    const ArmProtocol::ArmCommandMessage& message, fer_msgs::srv::ControlCommand::Request& request)
{
    using namespace ArmProtocol;
    request = fer_msgs::srv::ControlCommand::Request();
    request.frame_type = fer_msgs::srv::ControlCommand::Request::BASE;
    uint32_t expectedLength = 0;

    switch (message.header.msgId) {
    case EMERGENCY_STOP:
        request.command_type = "stop";
        break;
    case ARM_PARK:
        request.command_type = "park";
        break;
    case ARM_READY:
        request.command_type = "unpark";
        break;
    case ARM_DISABLE:
        request.command_type = "disable";
        break;
    case ARM_ENABLE:
        request.command_type = "enable";
        break;
    case ARM_RECOVER:
        request.command_type = "recover";
        break;
    case EFFECTOR_MOVE_ABS:
        expectedLength = sizeof(EffectorMoveAbs) - HEADER_SIZE;
        request.command_type = "move_cartesian";
        request.target_position
            = { message.effectorMoveAbs.x, message.effectorMoveAbs.y, message.effectorMoveAbs.z };
        request.target_orientation
            = { message.effectorMoveAbs.roll, message.effectorMoveAbs.pitch, message.effectorMoveAbs.yaw };
        break;
    case EFFECTOR_MOVE_REL:
        expectedLength = sizeof(EffectorMoveRel) - HEADER_SIZE;
        request.command_type = "move_cartesian";
        request.move_type = request.RELMOVE;
        request.frame_type = message.effectorMoveRel.frame;
        request.target_position = { message.effectorMoveRel.delta_x, message.effectorMoveRel.delta_y,
            message.effectorMoveRel.delta_z };
        request.target_orientation = { message.effectorMoveRel.delta_roll,
            message.effectorMoveRel.delta_pitch, message.effectorMoveRel.delta_yaw };
        break;
    case EFFECTOR_AXIS_MOVE_REL: {
        expectedLength = sizeof(EffectorAxisMoveRel) - HEADER_SIZE;
        int axis = message.effectorAxisMoveRel.axis_id;
        if (axis < 0 || axis >= MAX_AXIS_ID) {
            return false;
        }
        request.command_type = "move_cartesian";
        request.move_type = request.RELMOVE;
        request.frame_type = message.effectorAxisMoveRel.frame;
        if (axis < 3) {
            request.target_position[axis] = message.effectorAxisMoveRel.delta_axis;
        } else {
            request.target_orientation[axis - 3] = message.effectorAxisMoveRel.delta_axis;
        }
        break;
    }
    case JOINTS_MOVE_ABS: {
        expectedLength = sizeof(JointsMoveAbs) - HEADER_SIZE;
        request.command_type = "move_joints_pos";
        // Packed doubles may be unaligned; copy the payload before reading the joint values.
        std::array<double, ARM_JOINTS> joints;
        std::memcpy(joints.data(), reinterpret_cast<const char*>(&message) + HEADER_SIZE, sizeof(joints));
        request.joint_setpoint.assign(joints.begin(), joints.end());
        break;
    }
    case JOINT_MOVE_ABS:
        expectedLength = sizeof(JointMoveAbs) - HEADER_SIZE;
        request.command_type = "move_joint_i_pos";
        request.joint_index = message.jointMoveAbs.id;
        request.joint_setpoint = { message.jointMoveAbs.qi };
        break;
    case JOINT_MOVE_REL:
        expectedLength = sizeof(JointMoveRel) - HEADER_SIZE;
        request.command_type = "move_joint_i_pos";
        request.move_type = request.RELMOVE;
        request.joint_index = message.jointMoveRel.id;
        request.joint_setpoint = { message.jointMoveRel.delta_qi };
        break;
    case JOINT_MOVE_VEL:
        expectedLength = sizeof(JointMoveVel) - HEADER_SIZE;
        request.command_type = "move_joint_i_vel";
        request.joint_index = message.jointMoveVel.id;
        request.joint_setpoint = { message.jointMoveVel.velocity };
        break;
    case GRIPPER_MOVE:
        expectedLength = sizeof(GripperMove) - HEADER_SIZE;
        request.command_type = "gripper_move";
        request.gripper_setpoint = message.gripperMove.percentage * 0.08 / 100.0;
        if (request.gripper_setpoint < 0.0 || request.gripper_setpoint > 0.08) {
            return false;
        }
        break;
    case GRIPPER_OPEN:
        request.command_type = "gripper_move";
        request.gripper_setpoint = 0.08;
        break;
    case GRIPPER_CLOSE:
        request.command_type = "gripper_move";
        request.gripper_setpoint = 0.0;
        break;
    case GRIPPER_GRASP:
        expectedLength = sizeof(GripperGrasp) - HEADER_SIZE;
        request.command_type = "gripper_grasp";
        request.grasp_force = message.grasp.force;
        if (request.grasp_force < 5.0 || request.grasp_force > 70.0) {
            return false;
        }
        break;
    default:
        return false;
    }

    auto isFinite = [](double value) { return std::isfinite(value); };
    return message.header.length == expectedLength && request.joint_index >= 0
        && request.joint_index < static_cast<int>(ARM_JOINTS)
        && (request.frame_type == request.BASE || request.frame_type == request.EE)
        && std::all_of(request.joint_setpoint.begin(), request.joint_setpoint.end(), isFinite)
        && std::all_of(request.target_position.begin(), request.target_position.end(), isFinite)
        && std::all_of(request.target_orientation.begin(), request.target_orientation.end(), isFinite)
        && isFinite(request.gripper_setpoint) && isFinite(request.grasp_force);
}

bool ArmCommandParser::ParseAndSendCommand(const ArmProtocol::ArmCommandMessage& message)
{
    auto request = std::make_shared<fer_msgs::srv::ControlCommand::Request>();
    if (!ParseCommand(message, *request)) {
        RCLCPP_WARN(node_->get_logger(), "[Cmd Parser] Unsupported command or invalid payload: %u",
            message.header.msgId);
        return false;
    }
    if (!controlCommandClient_->service_is_ready() || pendingCommands_.size() >= MAX_PENDING_COMMANDS) {
        RCLCPP_WARN(node_->get_logger(), "[Cmd Parser] Controller unavailable or command queue full");
        return false;
    }
    pendingCommands_.push_back(
        { controlCommandClient_->async_send_request(request), node_->now().seconds() });
    return true;
}

void ArmCommandParser::CheckResponses()
{
    for (auto command = pendingCommands_.begin(); command != pendingCommands_.end();) {
        if (command->future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            auto response = command->future.get();
            if (response->success) {
                RCLCPP_INFO(node_->get_logger(), "[Cmd Parser] %s", response->res.c_str());
            } else if (response->res.find(" is not allowed in state ") != std::string::npos) {
                const std::string message = std::string(tc::yellow) + response->res + tc::none;
                RCLCPP_WARN(node_->get_logger(), "[Cmd Parser] %s", message.c_str());
            } else {
                RCLCPP_WARN(node_->get_logger(), "[Cmd Parser] %s", response->res.c_str());
            }
            command = pendingCommands_.erase(command);
        } else if (node_->now().seconds() - command->sendTime > COMMAND_RESPONSE_TIMEOUT) {
            controlCommandClient_->remove_pending_request(command->future);
            RCLCPP_WARN(node_->get_logger(), "[Cmd Parser] Controller service timed out");
            command = pendingCommands_.erase(command);
        } else {
            ++command;
        }
    }
}

}
