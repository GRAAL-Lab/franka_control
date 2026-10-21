#ifndef FER_CTRL_GENERIC_STATE_HPP
#define FER_CTRL_GENERIC_STATE_HPP

#include "fer_ctrl/ctrl_defines.hpp"
#include <fsm/fsm.h>
#include <ikcl/ikcl.h>

namespace fer {
namespace states {

class GenericState : public fsm::BaseState {
public:
    GenericState(const std::string& stateName, const std::string& actionName = "Idle");
    virtual ~GenericState() = default;

    std::shared_ptr<RobotInfo> robotInfo;
    std::shared_ptr<KCLConfiguration> conf;
    std::shared_ptr<rml::RobotModel> robotModel;
    std::shared_ptr<FrankaArmModel> armModel;
    std::shared_ptr<tpik::ActionManager> actionManager;
    TasksMap tasksMap;

    void ConfigureStateFromFile(libconfig::Config& configuration);
    fsm::retval OnEntry() override;

protected:
    fsm::retval ChangeState(const std::string& nextState, const std::string& message);
    std::string stateName_;
    std::string actionName_;
};

}
}

#endif
