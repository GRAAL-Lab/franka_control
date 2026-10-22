/** @file franka_driver.hpp */

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

/**
 * @brief Hardware driver with a dedicated libfranka control thread.
 * @details ROS callbacks exchange complete command/feedback records with that thread
 * through LatestValue buffers. Hardware watchdogs use steady wall time.
 */
class FrankaDriver : public rclcpp::Node {
public:
    /** @brief Load settings, create ROS interfaces and start the robot connection/control thread. */
    FrankaDriver();
    /** @brief Request a controlled shutdown and join the control thread before releasing the robot. */
    ~FrankaDriver() override;

private:
    /**
     * @brief Fixed-size command handed from the ROS executor to the control thread; receive time is
     * steady seconds.
     */
    struct ControlReference {
        std::array<double, ARM_JOINTS> position {};
        std::array<double, ARM_JOINTS> velocity {};
        std::array<double, ARM_JOINTS> acceleration {};
        double receivedTime = 0.0;
        bool enabled = false;
    };

    /** @brief Fixed-size robot/model snapshot handed from the control thread to the ROS executor. */
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

    /** @brief Read ROS parameters and dynamic controller settings; reject invalid rates/timeouts. */
    void LoadConfiguration();
    /** @brief Create ROS command/feedback interfaces and the wall-clock feedback publication timer. */
    void InitializeROSInterfaces();
    /** @brief Validate a dynamic reference and publish it to the control-thread command buffer. */
    void HandleDynamicControl(const fer_msgs::msg::DynamicControlData& message);
    /** @brief Queue enable/disable/recovery requests and return their acceptance status. */
    void HandleDriverCommand(fer_msgs::srv::DriverCommand::Request::SharedPtr request,
        fer_msgs::srv::DriverCommand::Response::SharedPtr response);
    /**
     * @brief Own the robot connection, process recovery and enter torque control when fresh commands
     * permit it.
     */
    void RunRobotThread();
    /** @brief Start libfranka torque control and translate exit conditions into persistent driver errors. */
    void RunControlLoop();
    /**
     * @brief Compute one libfranka torque command or a controlled stop.
     * @param state Measured robot state for this realtime cycle.
     * @param period Elapsed libfranka control period.
     * @return Bounded torques, marked MotionFinished when stopping completes or control fails.
     * @note Runs on the control thread; command and feedback exchange uses fixed-size buffers.
     */
    franka::Torques ComputeTorqueCommand(const franka::RobotState& state, franka::Duration period);
    /**
     * @brief Copy measured state and model dynamics into a complete feedback snapshot on the control
     * thread.
     */
    FeedbackSnapshot CaptureFeedback(const franka::RobotState& state, bool controlActive);
    /** @brief Consume the latest control-thread snapshot and publish ROS robot/joint state. */
    void PublishFeedback();
    /** @brief Latch an error, clear the enable request and store its message under the error mutex. */
    void SetError(const std::string& message);
    /**
     * @brief RunRobotThread queued libfranka error recovery on the control thread and update the
     * recovery/error flags.
     */
    void Recover();

    DCLConfiguration configuration_;
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
