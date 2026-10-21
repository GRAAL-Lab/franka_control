#include "fer_ctrl/tasks/obstacle_avoidance.hpp"

namespace fer {
namespace tasks {

ObstacleAvoidance::ObstacleAvoidance(const std::shared_ptr<rml::RobotModel>& robotModel,
    const std::string& toolFrame, const std::vector<PlaneObstacleConfiguration>& obstacles)
    : tpik::ReactiveTask(ID::tasks::obstacleAvoidance, static_cast<int>(obstacles.size()), robotModel->Dof(),
          tpik::TaskOption::Default)
    , robotModel_(robotModel)
    , toolFrame_(toolFrame)
    , obstacles_(obstacles)
{
    for (auto& obstacle : obstacles_) {
        double normalLength = obstacle.normal.norm();
        obstacle.normal /= normalLength;
        obstacle.offset /= normalLength;
    }
}

void ObstacleAvoidance::Update()
{
    Eigen::Vector3d toolPosition = robotModel_->TransformationMatrix(toolFrame_).TranslationVector();
    for (std::size_t plane = 0; plane < obstacles_.size(); ++plane) {
        x_[plane] = obstacles_[plane].normal.dot(toolPosition) + obstacles_[plane].offset;
    }
    tpik::ReactiveTask::Update();
}

void ObstacleAvoidance::UpdateJacobian()
{
    Eigen::MatrixXd toolJacobian = robotModel_->CartesianJacobian(toolFrame_);
    for (std::size_t plane = 0; plane < obstacles_.size(); ++plane) {
        J_.row(plane) = obstacles_[plane].normal.transpose() * toolJacobian.topRows(3);
    }
}

}
}
