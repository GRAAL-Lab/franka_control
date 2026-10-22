/** @file dynamic_control_node.hpp */

#ifndef FER_CTRL_DYNAMIC_CONTROL_NODE_HPP
#define FER_CTRL_DYNAMIC_CONTROL_NODE_HPP

#include "fer_ctrl/dynamic_controller.hpp"
#include "fer_msgs/msg/control_data.hpp"
#include "fer_msgs/msg/dynamic_control_data.hpp"
#include "fer_msgs/msg/state_data.hpp"
#include <rclcpp/rclcpp.hpp>

namespace fer {

/** @brief ROS-clock reference generator between kinematic velocities and driver torque control. */
class DynamicControlNode : public rclcpp::Node {
public:
    /** @brief Load settings and create ROS interfaces and the reference generator. */
    DynamicControlNode();

private:
    /** @brief Accept connected, finite joint feedback in the configured base frame. */
    void HandleStateData(const fer_msgs::msg::StateData& message);
    /** @brief Accept bounded kinematic velocity requests and record their node-clock receive time. */
    void HandleControlData(const fer_msgs::msg::ControlData& message);
    /**
     * @brief Advance references using elapsed node time, or reset them when feedback/control is
     * unavailable.
     */
    void Run();
    /** @brief Publish joint references and the current enable/status decision. */
    void PublishControl(bool enabled, const std::string& status);

    DCLConfiguration configuration_;
    std::unique_ptr<JointReferenceGenerator> referenceGenerator_;
    JointVector jointPosition_ = HomeJointPosition();
    JointVector requestedVelocity_ = JointVector::Zero();
    std::string baseFrame_;
    std::string controllerState_ = "disabled";
    double lastFeedbackTime_ = 0.0;
    double lastCommandTime_ = 0.0;
    double lastRunTime_ = 0.0;
    bool receivingFeedback_ = false;
    bool receivingCommands_ = false;
    bool controlRequested_ = false;
    bool controlActive_ = false;
    bool driverError_ = false;

    rclcpp::Subscription<fer_msgs::msg::StateData>::SharedPtr stateDataSub_;
    rclcpp::Subscription<fer_msgs::msg::ControlData>::SharedPtr controlDataSub_;
    rclcpp::Publisher<fer_msgs::msg::DynamicControlData>::SharedPtr dynamicControlPub_;
    rclcpp::TimerBase::SharedPtr runTimer_;
};

}

#endif
