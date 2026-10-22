/** @file simulator.hpp */

#ifndef FER_SIM_SIMULATOR_HPP
#define FER_SIM_SIMULATOR_HPP

#include "fer_ctrl/dynamic_controller.hpp"
#include "fer_msgs/msg/dynamic_control_data.hpp"
#include "fer_msgs/msg/state_data.hpp"
#include "fer_msgs/msg/gripper_state.hpp"
#include "fer_msgs/srv/driver_command.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

namespace fer {

/**
 * @brief Numerical arm/gripper driver and owner of the global ROS simulation clock.
 * @details Each wall-timer tick advances a fixed simulation step. simTimeScale changes
 * the wall period, so physics, watchdogs and ROS-clock consumers advance together.
 */
class Simulator : public rclcpp::Node {
public:
    /**
     * @brief Load simulation parameters and dynamic settings, then create driver services and the clock
     * timer.
     */
    Simulator();

private:
    /** @brief Load and validate isSim, simTimeScale, joint physics, initial posture, rates and frames. */
    void LoadConfiguration();
    /** @brief Create simulated driver interfaces and the wall timer that advances the simulation clock. */
    void InitializeROSInterfaces();
    /** @brief Validate a dynamic reference and record its arrival on the simulator-owned clock. */
    void HandleDynamicControl(const fer_msgs::msg::DynamicControlData& message);
    /** @brief Accept enable/disable/recovery according to the simulated driver state. */
    void HandleDriverCommand(fer_msgs::srv::DriverCommand::Request::SharedPtr request,
        fer_msgs::srv::DriverCommand::Response::SharedPtr response);
    /** @brief Validate and store a gripper target in metres and a speed in m/s. */
    void HandleGripperCommand(fer_msgs::srv::DriverCommand::Request::SharedPtr request,
        fer_msgs::srv::DriverCommand::Response::SharedPtr response);
    /**
     * @brief Advance one fixed simulation step, update the arm/gripper and publish clock and due state
     * messages.
     */
    void Run();
    /** @brief Process enable, watchdog and stop transitions at the supplied simulation time in seconds. */
    void UpdateControlState(double currentTime);
    /** @brief Integrate active arm control using substeps no longer than one simulated millisecond. */
    void AdvanceArm(double sampleTime);
    /**
     * @brief Integrate one bounded torque-control physics substep.
     * @param sampleTime Simulated step duration in seconds; nonpositive steps do nothing.
     */
    void IntegrateArmStep(double sampleTime);
    /**
     * @brief Update RML kinematics and publish arm, joint and gripper feedback at the current simulation
     * time.
     */
    void PublishState();
    /** @brief Latch a simulated driver error and clear the control request until explicit recovery. */
    void SetError(const std::string& message);

    FrankaArmModel model_;
    DCLConfiguration configuration_;
    std::unique_ptr<DynamicController> dynamicController_;
    std::unique_ptr<JointReferenceGenerator> stopReferenceGenerator_;
    JointReference jointReference_;
    JointVector jointPosition_ = HomeJointPosition();
    JointVector jointVelocity_ = JointVector::Zero();
    JointVector commandedTorque_ = JointVector::Zero();
    JointVector jointInertia_ = JointVector::Ones();
    JointVector jointDamping_ = JointVector::Zero();
    JointMatrix massMatrix_ = JointMatrix::Identity();

    std::string baseFrame_;
    std::string errorMessage_;
    double loopRate_ = 1000.0;
    double publishRate_ = 100.0;
    double commandTimeout_ = 0.2;
    double lastCommandTime_ = 0.0;
    double simTimeScale_ = 1.0;
    rclcpp::Time simulationTime_{ 0, 0, RCL_ROS_TIME };
    double lastPublishTime_ = 0.0;
    double stopStartTime_ = 0.0;
    bool controlRequested_ = false;
    bool commandEnabled_ = false;
    bool controlActive_ = false;
    bool stopping_ = false;
    bool hasError_ = false;

    double gripperWidth_ = 0.08;
    double gripperTarget_ = 0.08;
    double gripperSpeed_ = 0.05;

    rclcpp::Subscription<fer_msgs::msg::DynamicControlData>::SharedPtr dynamicControlSub_;
    rclcpp::Publisher<fer_msgs::msg::StateData>::SharedPtr stateDataPub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePub_;
    rclcpp::Publisher<fer_msgs::msg::GripperState>::SharedPtr gripperStatePub_;
    rclcpp::Service<fer_msgs::srv::DriverCommand>::SharedPtr driverCommandService_;
    rclcpp::Service<fer_msgs::srv::DriverCommand>::SharedPtr gripperCommandService_;
    rclcpp::TimerBase::SharedPtr runTimer_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clockPub_;
};

}

#endif
