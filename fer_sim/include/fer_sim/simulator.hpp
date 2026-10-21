#ifndef FER_SIM_SIMULATOR_HPP
#define FER_SIM_SIMULATOR_HPP

#include "fer_ctrl/dynamic_controller.hpp"
#include "fer_msgs/msg/dynamic_control_data.hpp"
#include "fer_msgs/msg/state_data.hpp"
#include "fer_msgs/msg/gripper_state.hpp"
#include "fer_msgs/srv/driver_command.hpp"
#include <rclcpp/rclcpp.hpp>

namespace fer {

class Simulator : public rclcpp::Node {
public:
    Simulator();

private:
    void LoadConfiguration();
    void Initialization();
    void DynamicControlCB(const fer_msgs::msg::DynamicControlData& message);
    void DriverCommandsHandler(fer_msgs::srv::DriverCommand::Request::SharedPtr request,
        fer_msgs::srv::DriverCommand::Response::SharedPtr response);
    void GripperCommandsHandler(fer_msgs::srv::DriverCommand::Request::SharedPtr request,
        fer_msgs::srv::DriverCommand::Response::SharedPtr response);
    void Run();
    void RunDynamicControl(double sampleTime);
    void PublishState();
    void SetError(const std::string& message);

    PandaModel model_;
    DCLConfiguration conf_;
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
    double lastRunTime_ = 0.0;
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
    rclcpp::TimerBase::SharedPtr publishTimer_;
};

}

#endif
