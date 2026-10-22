#include "fer_ctrl/states/state_move_cartesian.hpp"

namespace fer {
namespace states {

MoveCartesian::MoveCartesian()
    : GenericState(ID::states::move_cartesian, "MoveTool")
{
}

bool MoveCartesian::SetCartesianGoal(const CommandRequest& request)
{
    bool relative = request.moveType == "relative";
    if ((!relative && request.moveType != "absolute") || !request.targetPosition.allFinite()
        || !request.targetOrientation.ToVector().allFinite()
        || (request.frameType != 0 && request.frameType != 1)
        || (!relative && request.frameType != 1)) {
        robotInfo->status = "Use finite xyz/rpy, absolute BASE, or relative BASE/EE";
        return false;
    }

    Eigen::TransformationMatrix requestedFrame
        = MakeTransformation(request.targetPosition, request.targetOrientation);
    Eigen::TransformationMatrix goalFrame = requestedFrame;
    if (relative && request.frameType == 0) {
        goalFrame = robotInfo->feedback.toolFrame * requestedFrame;
    } else if (relative) {
        goalFrame = robotInfo->feedback.toolFrame;
        goalFrame.TranslationVector(goalFrame.TranslationVector() + requestedFrame.TranslationVector());
        goalFrame.RotationMatrix(requestedFrame.RotationMatrix() * goalFrame.RotationMatrix());
    }
    robotInfo->cartesianGoalFrame = goalFrame;
    robotInfo->stateEntryTime = robotInfo->currentTime;
    return true;
}

fsm::retval MoveCartesian::Execute()
{
    Eigen::Vector6d error = rml::CartesianError(robotInfo->feedback.toolFrame, robotInfo->cartesianGoalFrame);
    robotInfo->linearError = error.LinearVector().norm();
    robotInfo->angularError = error.AngularVector().norm();
    auto distanceTask
        = std::dynamic_pointer_cast<ikcl::CartesianDistance>(tasksMap.at(ID::tasks::toolDistance).task);
    auto orientationTask
        = std::dynamic_pointer_cast<ikcl::CartesianOrientation>(tasksMap.at(ID::tasks::toolOrientation).task);
    distanceTask->SetTargetDistance(error.LinearVector(), rml::FrameID::WorldFrame);
    orientationTask->SetTargetFrame(robotInfo->cartesianGoalFrame, rml::FrameID::WorldFrame);

    if (robotInfo->linearError < conf->linearErrorThreshold
        && robotInfo->angularError < conf->angularErrorThreshold) {
        return ChangeState(ID::states::idle, "Cartesian goal reached");
    }
    if (robotInfo->currentTime - robotInfo->stateEntryTime > conf->moveTimeout) {
        return ChangeState(ID::states::idle, "Cartesian move timed out");
    }
    return fsm::ok;
}

}
}
