/** @file state_move_cartesian.hpp */

#ifndef FER_CTRL_STATE_MOVE_CARTESIAN_HPP
#define FER_CTRL_STATE_MOVE_CARTESIAN_HPP

#include "fer_ctrl/states/generic_state.hpp"

namespace fer {
namespace states {

/** @brief Track a tool pose using RML Cartesian distance and orientation tasks. */
class MoveCartesian : public GenericState {
public:
    /** @brief Create the state with its default action and completion behavior. */
    MoveCartesian();
    /**
     * @brief Resolve an absolute base-frame or relative base/tool-frame goal.
     * @param request Translation in metres and RPY in radians.
     * @return True on acceptance; otherwise shared status explains the rejection.
     */
    bool SetCartesianGoal(const CommandRequest& request);
    /**
     * @brief Update Cartesian tasks and return to idle on convergence or moveTimeout.
     * @return FSM execution or queued-transition result.
     */
    fsm::retval Execute() override;
};

}
}

#endif
