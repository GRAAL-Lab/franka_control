#ifndef FER_TCP_BRIDGE_ARMPROTOCOL_H
#define FER_TCP_BRIDGE_ARMPROTOCOL_H

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>

namespace ArmProtocol {

enum FrameType : int32_t { EE = 0, BASE, LOCAL, WORLD };
enum ArmStatusEnum : int32_t {
    DISABLED = 0,
    UNPARKING,
    HOMING,
    IDLE,
    MOVING_EE,
    MOVING_JOINTS,
    PARKING,
    PARKED,
    FAULT,
    ENABLING,
    MAX_ARM_STATUS_ENUM
};
enum AxisId : int32_t { X_AXIS = 0, Y_AXIS, Z_AXIS, ROLL_AXIS, PITCH_AXIS, YAW_AXIS, MAX_AXIS_ID };
enum CmdMsgId : uint32_t {
    EMERGENCY_STOP = 0,
    ARM_HOMING,
    ARM_PARK,
    ARM_READY,
    ARM_DISABLE,
    ARM_ENABLE,
    EFFECTOR_MOVE_ABS,
    EFFECTOR_MOVE_REL,
    EFFECTOR_AXIS_MOVE_REL,
    JOINTS_MOVE_ABS,
    JOINT_MOVE_ABS,
    JOINT_MOVE_REL,
    JOINT_MOVE_VEL,
    GRIPPER_MOVE,
    GRIPPER_OPEN,
    GRIPPER_CLOSE,
    GRIPPER_GRASP,
    GRIPPER_LOCK,
    ARM_RECOVER,
    MAX_CMD_MSG_ID
};
enum RptMsgId : uint32_t {
    ARM_STATUS = 0,
    JOINTS_POSITION,
    EFFECTOR_POSITION,
    MOTORS_CURRENT,
    LIMIT_SWITCHES_STATE,
    GRIPPER_LOCKED,
    MAX_RPT_MSG_ID
};

// Undroid framing: native little-endian integers/doubles, with no struct padding.
// length is the payload size, excluding the 12-byte header. Angles use radians.
#pragma pack(push, 1)
struct ArmProtocolHeader {
    uint32_t destination;
    uint32_t msgId;
    uint32_t length;
};
struct ArmStatusCode {
    int32_t code;
};
struct EffectorMoveAbs {
    ArmProtocolHeader header;
    double x, y, z, roll, pitch, yaw;
};
struct EffectorMoveRel {
    ArmProtocolHeader header;
    FrameType frame;
    double delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw;
};
struct EffectorAxisMoveRel {
    ArmProtocolHeader header;
    int32_t frame;
    int32_t axis_id;
    double delta_axis;
};
struct JointsMoveAbs {
    ArmProtocolHeader header;
    double q1, q2, q3, q4, q5, q6, q7;
};
struct JointMoveAbs {
    ArmProtocolHeader header;
    int32_t id;
    double qi;
};
struct JointMoveRel {
    ArmProtocolHeader header;
    int32_t id;
    double delta_qi;
};
struct JointMoveVel {
    ArmProtocolHeader header;
    int32_t id;
    double velocity;
};
struct GripperMove {
    ArmProtocolHeader header;
    double percentage;
};
struct GripperGrasp {
    ArmProtocolHeader header;
    double force; // Franka grasp force in N; Undroid used a motor-current threshold.
};
struct ArmStatus {
    ArmProtocolHeader header;
    ArmStatusCode status;
};
struct JointsPosition {
    ArmProtocolHeader header;
    double q1, q2, q3, q4, q5, q6, q7;
    double pincers; // Opening percentage, as in Undroid.
};
struct EffectorPosition {
    ArmProtocolHeader header;
    double x, y, z, roll, pitch, yaw;
};
struct MotorsCurrent {
    ArmProtocolHeader header;
    double q1_current, q2_current, q3_current, q4_current, q5_current, q6_current, q7_current;
    // The original report ID is retained; FER reports joint torque in Nm.
};
struct GripperLocked {
    ArmProtocolHeader header;
    int32_t isLocked;
};
union ArmCommandMessage {
    ArmProtocolHeader header;
    GripperGrasp grasp;
    GripperMove gripperMove;
    JointMoveVel jointMoveVel;
    JointMoveRel jointMoveRel;
    JointMoveAbs jointMoveAbs;
    JointsMoveAbs jointsMoveAbs;
    EffectorAxisMoveRel effectorAxisMoveRel;
    EffectorMoveRel effectorMoveRel;
    EffectorMoveAbs effectorMoveAbs;
};
union ArmReportMessage {
    ArmProtocolHeader header;
    EffectorPosition effectorPosition;
    JointsPosition jointsPosition;
    ArmStatus armStatus;
    MotorsCurrent motorCurrent;
    GripperLocked gripperLocked;
};
#pragma pack(pop)

inline constexpr std::size_t HEADER_SIZE = sizeof(ArmProtocolHeader);
inline constexpr std::size_t MAX_COMMAND_PAYLOAD = sizeof(ArmCommandMessage) - HEADER_SIZE;
inline constexpr std::size_t MAX_REPORT_PAYLOAD = sizeof(ArmReportMessage) - HEADER_SIZE;
static_assert(HEADER_SIZE == 12 && sizeof(double) == 8);
static_assert(sizeof(JointsMoveAbs) == 68 && sizeof(EffectorMoveRel) == 64);

inline const std::map<CmdMsgId, std::string> CmdMsgIdStringMap = { { EMERGENCY_STOP, "EMERGENCY_STOP" },
    { ARM_HOMING, "ARM_HOMING" }, { ARM_PARK, "ARM_PARK" }, { ARM_READY, "ARM_READY" },
    { ARM_DISABLE, "ARM_DISABLE" }, { ARM_ENABLE, "ARM_ENABLE" }, { EFFECTOR_MOVE_ABS, "EFFECTOR_MOVE_ABS" },
    { EFFECTOR_MOVE_REL, "EFFECTOR_MOVE_REL" }, { EFFECTOR_AXIS_MOVE_REL, "EFFECTOR_AXIS_MOVE_REL" },
    { JOINTS_MOVE_ABS, "JOINTS_MOVE_ABS" }, { JOINT_MOVE_ABS, "JOINT_MOVE_ABS" },
    { JOINT_MOVE_REL, "JOINT_MOVE_REL" }, { JOINT_MOVE_VEL, "JOINT_MOVE_VEL" },
    { GRIPPER_MOVE, "GRIPPER_MOVE" }, { GRIPPER_OPEN, "GRIPPER_OPEN" }, { GRIPPER_CLOSE, "GRIPPER_CLOSE" },
    { GRIPPER_GRASP, "GRIPPER_GRASP" }, { GRIPPER_LOCK, "GRIPPER_LOCK" }, { ARM_RECOVER, "ARM_RECOVER" } };

inline const std::map<RptMsgId, std::string> RptMsgIdStringMap
    = { { ARM_STATUS, "ARM_STATUS" }, { JOINTS_POSITION, "JOINTS_POSITION" },
          { EFFECTOR_POSITION, "EFFECTOR_POSITION" }, { MOTORS_CURRENT, "MOTORS_CURRENT" },
          { LIMIT_SWITCHES_STATE, "LIMIT_SWITCHES_STATE" }, { GRIPPER_LOCKED, "GRIPPER_LOCKED" } };

}

#endif
