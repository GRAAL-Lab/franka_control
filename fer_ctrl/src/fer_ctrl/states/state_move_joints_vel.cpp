#include "fer_ctrl/states/state_move_joints_vel.hpp"

namespace fer {
namespace states {

MoveJointsVel::MoveJointsVel()
    : GenericState(ID::states::move_joints_vel, "MoveJointsVelocity")
{
}

bool MoveJointsVel::SetJointsGoal(const CommandRequest& request)
{
    bool singleJoint = request.commandType == "move_joint_i_vel";
    if (request.jointSetpoint.size() != (singleJoint ? 1 : ARM_JOINTS)
        || (singleJoint && (request.jointIndex < 0 || request.jointIndex >= 7))) {
        robotInfo->status = "Expected seven velocities, or one velocity and an index in 0..6";
        return false;
    }

    JointVector velocityGoal = JointVector::Zero();
    for (std::size_t index = 0; index < request.jointSetpoint.size(); ++index) {
        int joint = singleJoint ? request.jointIndex : static_cast<int>(index);
        velocityGoal[joint] = request.jointSetpoint[index];
    }
    if (!velocityGoal.allFinite() || (velocityGoal.array() < conf->saturationMin.array()).any()
        || (velocityGoal.array() > conf->saturationMax.array()).any()) {
        robotInfo->status = "Velocity setpoints must be finite and within saturationMin/saturationMax";
        return false;
    }

    robotInfo->velocityGoal = velocityGoal;
    robotInfo->lastVelocityCommandTime = robotInfo->currentTime;
    return true;
}

fsm::retval MoveJointsVel::Execute()
{
    auto task = std::dynamic_pointer_cast<ikcl::JointsVelocity>(tasksMap.at(ID::tasks::jointsVelocity).task);
    task->SetReferenceRate(robotInfo->velocityGoal);
    if (robotInfo->currentTime - robotInfo->lastVelocityCommandTime > conf->velocityCommandTimeout) {
        robotInfo->velocityGoal.setZero();
        task->SetReferenceRate(robotInfo->velocityGoal);
        return ChangeState(ID::states::idle, "Velocity command expired");
    }
    return fsm::ok;
}

}
}
