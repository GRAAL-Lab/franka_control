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

struct CommandRequest {
    std::string commandType;
    std::string moveType = "absolute";
    std::vector<double> jointSetpoint;
    int jointIndex = 0;
    Eigen::Vector3d targetPosition = Eigen::Vector3d::Zero();
    rml::EulerRPY targetOrientation;
    int frameType = 1;
};

struct CommandResult {
    bool success = false;
    std::string message;
};

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

struct PlaneObstacleConfiguration {
    std::string name;
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    double offset = -0.1;
};

struct KCLConfiguration {
    double loopRate = 100.0;
    double linearErrorThreshold = 0.002;
    double angularErrorThreshold = 0.01;
    double jointErrorThreshold = 0.01;
    double feedbackTimeout = 0.2;
    double velocityCommandTimeout = 0.3;
    double moveTimeout = 30.0;
    double enableTimeout = 5.0;
    double jointLimitMargin = 0.05;
    double actionTransitionDuration = 0.15;
    JointVector saturationMin = JointVector::Constant(-0.35);
    JointVector saturationMax = JointVector::Constant(0.35);
    JointVector parkedJointsPos = (JointVector() << 0.0, -0.4, 0.0, -2.7, 0.0, 2.3, PI / 4.0).finished();
    JointVector unparkedJointsPos = HomeJointPosition();
    std::vector<PlaneObstacleConfiguration> obstacles;

    void ConfigureFromFile(libconfig::Config& configuration);
};

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

struct TaskInfo {
    std::shared_ptr<tpik::Task> task;
};

using TasksMap = std::unordered_map<std::string, TaskInfo>;

struct ControlOutput {
    JointVector jointVelocity = JointVector::Zero();
    double jointError = 0.0;
    double linearError = 0.0;
    double angularError = 0.0;
    bool jointLimitActive = false;
};

}

#endif
