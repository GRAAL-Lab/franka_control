#ifndef FER_DRIVER_FRANKA_DRIVER_HPP
#define FER_DRIVER_FRANKA_DRIVER_HPP

#include "fer_ctrl/dynamic_controller.hpp"
#include "fer_driver/latest_value.hpp"
#include "fer_msgs/msg/dynamic_control_data.hpp"
#include "fer_msgs/msg/state_data.hpp"
#include "fer_msgs/srv/driver_command.hpp"

#include <franka/robot.h>
#include <franka/model.h>
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <mutex>
#include <thread>

namespace fer {

class FrankaDriver : public rclcpp::Node {
public:
    FrankaDriver();
    ~FrankaDriver() override;

private:
    struct ControlReference {
        std::array<double, ARM_JOINTS> position {};
        std::array<double, ARM_JOINTS> velocity {};
        std::array<double, ARM_JOINTS> acceleration {};
        double receivedTime = 0.0;
        bool enabled = false;
    };

    struct FeedbackSnapshot {
        std::array<double, ARM_JOINTS> position {};
        std::array<double, ARM_JOINTS> velocity {};
        std::array<double, ARM_JOINTS> measuredTorque {};
        std::array<double, ARM_JOINTS> commandedTorque {};
        std::array<double, ARM_JOINTS> coriolis {};
        std::array<double, 49> massMatrix {};
        std::array<double, 16> toolFrame {};
        std::array<double, 16> flangeToTool {};
        std::array<double, 42> toolJacobian {};
        double successRate = 0.0;
        bool connected = false;
        bool controlActive = false;
        bool ready = false;
        bool hasError = false;
        uint8_t robotMode = 0;
    };

    void LoadConfiguration();
    void Initialization();
    void DynamicControlCB(const fer_msgs::msg::DynamicControlData& message);
    void CommandsHandler(fer_msgs::srv::DriverCommand::Request::SharedPtr request,
        fer_msgs::srv::DriverCommand::Response::SharedPtr response);
    void Run();
    void RunControlLoop();
    franka::Torques TorqueControlCB(const franka::RobotState& state, franka::Duration period);
    FeedbackSnapshot ReadSnapshot(const franka::RobotState& state, bool controlActive);
    void PublishFeedback();
    void SetError(const std::string& message);
    void Recover();

    DCLConfiguration conf_;
    std::unique_ptr<DynamicController> dynamicController_;
    std::unique_ptr<JointReferenceGenerator> stopReferenceGenerator_;
    std::unique_ptr<franka::Robot> robot_;
    std::unique_ptr<franka::Model> model_;

    std::string robotAddress_;
    std::string baseFrame_;
    double commandTimeout_ = 0.2;
    double publishRate_ = 100.0;
    bool enforceRealtime_ = true;

    std::atomic<bool> running_ { true };
    std::atomic<bool> enableRequested_ { false };
    std::atomic<bool> recoverRequested_ { false };
    std::atomic<bool> recovering_ { false };
    std::atomic<bool> hasError_ { false };
    std::atomic<bool> connected_ { false };
    std::atomic<bool> ready_ { false };
    std::atomic<bool> controlActive_ { false };

    ControlReference controlReference_;
    JointReference jointReference_;
    bool stopping_ = false;
    bool watchdogExpired_ = false;
    bool invalidDynamics_ = false;
    bool stopTimedOut_ = false;
    double stopStartTime_ = 0.0;

    LatestValue<ControlReference> commandBuffer_;
    LatestValue<FeedbackSnapshot> feedbackBuffer_;
    std::mutex errorMutex_;
    std::string errorMessage_;
    std::thread controlThread_;

    rclcpp::Subscription<fer_msgs::msg::DynamicControlData>::SharedPtr dynamicControlSub_;
    rclcpp::Publisher<fer_msgs::msg::StateData>::SharedPtr stateDataPub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePub_;
    rclcpp::Service<fer_msgs::srv::DriverCommand>::SharedPtr commandService_;
    rclcpp::TimerBase::SharedPtr publishTimer_;
};

}

#endif
