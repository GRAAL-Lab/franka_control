/** @file franka_tcp_bridge.hpp */

#ifndef FER_TCP_BRIDGE_FRANKA_TCP_BRIDGE_HPP
#define FER_TCP_BRIDGE_FRANKA_TCP_BRIDGE_HPP

#include "fer_tcp_bridge/arm_command_parser.hpp"
#include "fer_msgs/msg/control_data.hpp"
#include "fer_msgs/msg/gripper_state.hpp"
#include "fer_msgs/msg/state_data.hpp"
#include <boost/asio.hpp>

namespace fer {

/**
 * @brief Single-client nonblocking TCP server for the binary ArmProtocol.
 * @details Node-clock timers poll commands and reports. Partial frames and writes are
 * retained between cycles; bounded buffers disconnect clients that cannot keep up.
 */
class FrankaTCPBridge : public rclcpp::Node {
public:
    /** @brief Load connection parameters, bind the TCP listener and create ROS interfaces and timers. */
    FrankaTCPBridge();

private:
    /**
     * @brief Configure the node clock, open the listener and create ROS feedback subscriptions and
     * timers.
     */
    void InitializeROSInterfaces();
    /** @brief Accept a client, forward a bounded batch of commands and flush pending reports. */
    void CommandLoop();
    /** @brief Queue status, joint, tool and gripper reports when fresh ROS data is available. */
    void FeedbackLoop();
    /** @brief Translate the cached controller state to its stable protocol status code and queue it. */
    void QueueControlStatusReport();
    /** @brief Queue measured joint positions/torques and the base-frame tool pose as protocol reports. */
    void QueueRobotReports();
    /** @brief Queue the cached gripper grasp state as a protocol report. */
    void QueueGripperReport();
    /**
     * @brief Read available bytes and extract at most one complete command frame.
     * @return True when commandMessage_ contains a command; false if incomplete or disconnected.
     */
    bool ReadCommandFrame();
    /** @brief Append a framed report to the bounded send buffer; disconnect on overflow or invalid length. */
    void QueueReport(const ArmProtocol::ArmReportMessage& report);
    /** @brief Write available queued bytes without blocking and preserve any unsent remainder. */
    void SendQueuedReports();
    /** @brief Close the client socket and clear partial input/output buffers. */
    void Disconnect();
    /** @brief Cache robot feedback and mark it for the next protocol report cycle. */
    void HandleStateData(const fer_msgs::msg::StateData& message);
    /** @brief Cache controller state and mark it for the next status report. */
    void HandleControlData(const fer_msgs::msg::ControlData& message);
    /** @brief Cache gripper state and mark it for the next gripper report. */
    void HandleGripperState(const fer_msgs::msg::GripperState& message);

    boost::asio::io_context ioContext_;
    boost::asio::ip::tcp::acceptor acceptor_;
    boost::asio::ip::tcp::socket socket_;
    ArmCommandParser armCommandParser_;
    ArmProtocol::ArmCommandMessage commandMessage_ {};
    std::vector<uint8_t> receiveBuffer_;
    std::vector<uint8_t> sendBuffer_;
    std::size_t sendOffset_ = 0;
    bool connected_ = false;
    bool newFeedback_ = false;
    bool newControl_ = false;
    bool newGripper_ = false;
    fer_msgs::msg::StateData stateData_;
    fer_msgs::msg::ControlData controlData_;
    fer_msgs::msg::GripperState gripperState_;

    rclcpp::TimerBase::SharedPtr commandsTimer_;
    rclcpp::TimerBase::SharedPtr feedbackTimer_;
    rclcpp::Subscription<fer_msgs::msg::StateData>::SharedPtr stateDataSub_;
    rclcpp::Subscription<fer_msgs::msg::ControlData>::SharedPtr controlDataSub_;
    rclcpp::Subscription<fer_msgs::msg::GripperState>::SharedPtr gripperStateSub_;
};

}

#endif
