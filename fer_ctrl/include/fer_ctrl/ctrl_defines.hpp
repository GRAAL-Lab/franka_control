/** @file ctrl_defines.hpp */

#ifndef FER_CTRL_CTRL_DEFINES_HPP
#define FER_CTRL_CTRL_DEFINES_HPP

#include "fer_model/franka_arm_model.hpp"
#include <libconfig.h++>
#include <tpik/TPIKlib.h>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace fer {

namespace ID {
namespace states {
inline const std::string disabled = "disabled";
inline const std::string disabling = "disabling";
inline const std::string enabling = "enabling";
inline const std::string idle = "idle";
inline const std::string parked = "parked";
inline const std::string parking = "parking";
inline const std::string unparking = "unparking";
inline const std::string move_joints_pos = "move_joints_pos";
inline const std::string move_joints_vel = "move_joints_vel";
inline const std::string move_cartesian = "move_cartesian";
inline const std::string fault = "fault";
}
namespace tasks {
inline const std::string jointsLimit = "JointsLimit";
inline const std::string obstacleAvoidance = "ObstacleAvoidance";
inline const std::string jointsPosition = "JointsPosition";
inline const std::string jointsVelocity = "JointsVelocity";
inline const std::string jointsPosture = "JointsPosture";
inline const std::string toolDistance = "ToolCartesianDistance";
inline const std::string toolOrientation = "ToolCartesianOrientation";
}
}

/** @brief Controller command: joint angles use radians, Cartesian positions metres, and RPY radians. */
struct CommandRequest {
    std::string commandType;
    std::string moveType = "absolute";
    std::vector<double> jointSetpoint;
    /** @brief Zero-based joint index for a single-joint command. */
    int jointIndex = 0;
    Eigen::Vector3d targetPosition = Eigen::Vector3d::Zero();
    rml::EulerRPY targetOrientation;
    /** @brief Cartesian reference frame: 0 for the tool frame, 1 for the base frame. */
    int frameType = 1;
};

/** @brief Command acceptance result and an explanation suitable for the ROS service response. */
struct CommandResult {
    bool success = false;
    std::string message;
};

/** @brief Validated driver feedback; tool pose and Jacobian are expressed in the robot base frame. */
struct RobotFeedback {
    JointVector jointPosition = HomeJointPosition();
    JointVector jointVelocity = JointVector::Zero();
    Eigen::TransformationMatrix toolFrame;
    Eigen::TransformationMatrix flangeToTool;
    JacobianMatrix toolJacobian = JacobianMatrix::Zero();
    bool connected = false;
    bool ready = false;
    bool controlActive = false;
    bool hasError = false;
};

/** @brief Plane n.dot(position) + offset = 0 in the base frame; positive distance is the safe side. */
struct PlaneObstacleConfiguration {
    std::string name;
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    double offset = -0.1;
};

/**
 * @brief Kinematic control settings: rates in Hz, times in seconds, distances in metres and angles in
 * radians.
 */
struct KCLConfiguration {
    double loopRate = 100.0;
    double linearErrorThreshold = 0.002;
    double angularErrorThreshold = 0.01;
    double jointErrorThreshold = 0.01;
    double feedbackTimeout = 0.2;
    double velocityCommandTimeout = 0.3;
    double moveTimeout = 30.0;
    double enableTimeout = 5.0;
    double disableTimeout = 5.0;
    double jointLimitMargin = 0.05;
    double actionTransitionDuration = 0.15;
    JointVector saturationMin = JointVector::Constant(-0.35);
    JointVector saturationMax = JointVector::Constant(0.35);
    JointVector parkedJointsPos = (JointVector() << 0.0, -0.4, 0.0, -2.7, 0.0, 2.3, PI / 4.0).finished();
    JointVector unparkedJointsPos = HomeJointPosition();
    std::vector<PlaneObstacleConfiguration> obstacles;

    /**
     * @brief Read and validate kinematic settings, postures and obstacle planes.
     * @param configuration Parsed kcl_fer.conf settings.
     * @throws std::exception If required settings are missing or values are invalid.
     */
    void ConfigureFromFile(libconfig::Config& configuration);
};

/**
 * @brief State shared by the controller commands and FSM states.
 * @details All time fields use seconds from the clock supplied to TaskPriorityController.
 * Access is serialized by the owning controller; this structure does not provide locking.
 */
struct RobotInfo {
    RobotFeedback feedback;
    bool receivingFeedback = false;
    double currentTime = 0.0;
    double lastFeedbackTime = 0.0;
    double stateEntryTime = 0.0;
    double lastVelocityCommandTime = 0.0;
    std::string toolID = "Panda_Tool";
    std::string status = "Waiting for feedback and enable";
    JointVector jointsGoal = HomeJointPosition();
    JointVector velocityGoal = JointVector::Zero();
    Eigen::TransformationMatrix cartesianGoalFrame;
    double jointError = 0.0;
    double linearError = 0.0;
    double angularError = 0.0;
};

/** @brief Task instance registered under its configured task identifier. */
struct TaskInfo {
    std::shared_ptr<tpik::Task> task;
};

using TasksMap = std::unordered_map<std::string, TaskInfo>;

/** @brief Joint velocity command in rad/s and task errors in radians/metres for one control cycle. */
struct ControlOutput {
    JointVector jointVelocity = JointVector::Zero();
    double jointError = 0.0;
    double linearError = 0.0;
    double angularError = 0.0;
    bool jointLimitActive = false;
};

}

#endif
