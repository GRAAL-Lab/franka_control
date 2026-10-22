/** @file generic_state.hpp */

#ifndef FER_CTRL_GENERIC_STATE_HPP
#define FER_CTRL_GENERIC_STATE_HPP

#include "fer_ctrl/ctrl_defines.hpp"
#include <fsm/fsm.h>
#include <ikcl/ikcl.h>

namespace fer {
namespace states {

/** @brief FSM state base selecting a configured TPIK action and sharing controller dependencies. */
class GenericState : public fsm::BaseState {
public:
    /**
     * @brief Associate an FSM state with its default TPIK action.
     * @param stateName State identifier used in the controller configuration.
     * @param actionName Default action; the matching entry under states may override it.
     */
    GenericState(const std::string& stateName, const std::string& actionName = "Idle");
    /** @brief Destroy a state through its base interface. */
    virtual ~GenericState() = default;

    std::shared_ptr<RobotInfo> robotInfo;
    std::shared_ptr<KCLConfiguration> configuration;
    std::shared_ptr<rml::RobotModel> robotModel;
    std::shared_ptr<FrankaArmModel> armModel;
    std::shared_ptr<tpik::ActionManager> actionManager;
    TasksMap tasksMap;

    /**
     * @brief Load the optional state action override and verify that the action exists.
     * @param configuration Parsed controller configuration.
     * @throws std::exception If the selected TPIK action cannot be resolved.
     */
    void ConfigureStateFromFile(libconfig::Config& configuration);
    /**
     * @brief Record entry time and select the state action, blending for motion states.
     * @return fsm::ok after action selection.
     * @throws std::runtime_error If the action manager rejects the action.
     */
    fsm::retval OnEntry() override;

protected:
    /**
     * @brief Set shared status and queue an FSM transition.
     * @param nextState Target state identifier.
     * @param message Human-readable reason for the transition.
     * @return FSM transition result.
     */
    fsm::retval ChangeState(const std::string& nextState, const std::string& message);
    std::string stateName_;
    std::string actionName_;
};

}
}

#endif
