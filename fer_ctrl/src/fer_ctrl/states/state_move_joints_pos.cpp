#include "fer_ctrl/states/state_move_joints_pos.hpp"

namespace fer {
namespace states {

MoveJointsPos::MoveJointsPos()
    : MoveJointsPos(ID::states::move_joints_pos, "MoveJointsPosition", ID::states::idle)
{
}

MoveJointsPos::MoveJointsPos(
    const std::string& stateName, const std::string& actionName, const std::string& completionState)
    : GenericState(stateName, actionName)
    , completionState_(completionState)
{
}

bool MoveJointsPos::SetJointsGoal(const CommandRequest& request)
{
    bool singleJoint = request.commandType == "move_joint_i_pos";
    if (request.jointSetpoint.size() != (singleJoint ? 1 : ARM_JOINTS)
        || (singleJoint && (request.jointIndex < 0 || request.jointIndex >= 7))
        || (request.moveType != "absolute" && request.moveType != "relative")) {
        robotInfo->status
            = "Expected seven setpoints, or one setpoint and an index in 0..6; use absolute/relative";
        return false;
    }

    JointVector jointsGoal = robotInfo->feedback.jointPosition;
    for (std::size_t index = 0; index < request.jointSetpoint.size(); ++index) {
        int joint = singleJoint ? request.jointIndex : static_cast<int>(index);
        jointsGoal[joint] = request.jointSetpoint[index];
        if (request.moveType == "relative") {
            jointsGoal[joint] += robotInfo->feedback.jointPosition[joint];
        }
    }
    if (!JointsWithinLimits(jointsGoal, configuration->jointLimitMargin)) {
        robotInfo->status = "Joint goal must be finite and within the Panda limits and configured margin";
        return false;
    }

    robotInfo->jointsGoal = jointsGoal;
    robotInfo->stateEntryTime = robotInfo->currentTime;
    return true;
}

fsm::retval MoveJointsPos::Execute()
{
    auto task = std::dynamic_pointer_cast<ikcl::JointsPosition>(tasksMap.at(ID::tasks::jointsPosition).task);
    task->Reference() = robotInfo->jointsGoal;
    robotInfo->jointError = (robotInfo->jointsGoal - robotInfo->feedback.jointPosition).cwiseAbs().maxCoeff();

    if (robotInfo->jointError < configuration->jointErrorThreshold
        && robotInfo->feedback.jointVelocity.cwiseAbs().maxCoeff() < 0.03) {
        return ChangeState(completionState_, "Joint goal reached");
    }
    if (robotInfo->currentTime - robotInfo->stateEntryTime > configuration->moveTimeout) {
        return ChangeState(ID::states::idle, "Joint move timed out");
    }
    return fsm::ok;
}

}
}
