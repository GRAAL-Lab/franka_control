#ifndef FER_TCP_BRIDGE_FRANKA_TCP_BRIDGE_HPP
#define FER_TCP_BRIDGE_FRANKA_TCP_BRIDGE_HPP

#include "fer_tcp_bridge/arm_command_parser.hpp"
#include "fer_msgs/msg/control_data.hpp"
#include "fer_msgs/msg/gripper_state.hpp"
#include "fer_msgs/msg/state_data.hpp"
#include <boost/asio.hpp>

namespace fer {

class FrankaTCPBridge : public rclcpp::Node {
public:
    FrankaTCPBridge();

private:
    void Initialization();
    void CommandLoop();
    void FeedbackLoop();
    bool getArmCommandData();
    void sendArmReportData(const ArmProtocol::ArmReportMessage& report);
    void SendQueuedReports();
    void Disconnect();
    void StateDataCB(const fer_msgs::msg::StateData& message);
    void ControlDataCB(const fer_msgs::msg::ControlData& message);
    void GripperStateCB(const fer_msgs::msg::GripperState& message);

    boost::asio::io_context ioService_;
    boost::asio::ip::tcp::acceptor acceptor_;
    boost::asio::ip::tcp::socket socket_;
    ArmCommandParser armCommandParser_;
    ArmProtocol::ArmCommandMessage armCommandTCPMsg_ {};
    std::vector<uint8_t> receiveBuffer_;
    std::vector<uint8_t> sendBuffer_;
    std::size_t sendOffset_ = 0;
    bool connected_ = false;
    bool newFeedback_ = false;
    bool newControl_ = false;
    bool newGripper_ = false;
    fer_msgs::msg::StateData stateDataMsg_;
    fer_msgs::msg::ControlData controlDataMsg_;
    fer_msgs::msg::GripperState gripperStateMsg_;

    rclcpp::TimerBase::SharedPtr commandsTimer_;
    rclcpp::TimerBase::SharedPtr feedbackTimer_;
    rclcpp::Subscription<fer_msgs::msg::StateData>::SharedPtr stateDataSub_;
    rclcpp::Subscription<fer_msgs::msg::ControlData>::SharedPtr controlDataSub_;
    rclcpp::Subscription<fer_msgs::msg::GripperState>::SharedPtr gripperStateSub_;
};

}

#endif
