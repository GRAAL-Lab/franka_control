/** @file obstacle_avoidance.hpp */

#ifndef FER_CTRL_TASK_OBSTACLE_AVOIDANCE_HPP
#define FER_CTRL_TASK_OBSTACLE_AVOIDANCE_HPP

#include "fer_ctrl/ctrl_defines.hpp"

namespace fer {
namespace tasks {

/** @brief Reactive signed-distance task preserving the configured positive side of obstacle planes. */
class ObstacleAvoidance : public tpik::ReactiveTask {
public:
    /**
     * @brief Create one signed-distance task dimension for each configured plane.
     * @param robotModel RML model providing world-frame tool kinematics.
     * @param toolFrame Tool frame identifier in the RML model.
     * @param obstacles Validated planes with nonzero normals, expressed in the model world frame.
     */
    ObstacleAvoidance(const std::shared_ptr<rml::RobotModel>& robotModel, const std::string& toolFrame,
        const std::vector<PlaneObstacleConfiguration>& obstacles);
    /** @brief Refresh signed tool-to-plane distances, then update reactive activation and references. */
    void Update() override;

protected:
    /** @brief Project the translational tool Jacobian onto each normalized plane normal. */
    void UpdateJacobian() override;

private:
    std::shared_ptr<rml::RobotModel> robotModel_;
    std::string toolFrame_;
    std::vector<PlaneObstacleConfiguration> obstacles_;
};

}
}

#endif
