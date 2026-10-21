#ifndef FER_CTRL_TASK_OBSTACLE_AVOIDANCE_HPP
#define FER_CTRL_TASK_OBSTACLE_AVOIDANCE_HPP

#include "fer_ctrl/ctrl_defines.hpp"

namespace fer {
namespace tasks {

// Signed plane distances preserve the configured safe side, including on the plane.
class ObstacleAvoidance : public tpik::ReactiveTask {
public:
    ObstacleAvoidance(const std::shared_ptr<rml::RobotModel>& robotModel, const std::string& toolFrame,
        const std::vector<PlaneObstacleConfiguration>& obstacles);
    void Update() override;

protected:
    void UpdateJacobian() override;

private:
    std::shared_ptr<rml::RobotModel> robotModel_;
    std::string toolFrame_;
    std::vector<PlaneObstacleConfiguration> obstacles_;
};

}
}

#endif
