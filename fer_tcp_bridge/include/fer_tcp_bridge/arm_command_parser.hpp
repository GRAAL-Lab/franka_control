#ifndef FER_TCP_BRIDGE_ARM_COMMAND_PARSER_HPP
#define FER_TCP_BRIDGE_ARM_COMMAND_PARSER_HPP

#include "fer_tcp_bridge/armprotocol.h"
#include "fer_msgs/srv/control_command.hpp"
#include <rclcpp/rclcpp.hpp>

namespace fer {

class ArmCommandParser {
public:
    explicit ArmCommandParser(rclcpp::Node* node);

    static bool ParseCommand(
        const ArmProtocol::ArmCommandMessage& message, fer_msgs::srv::ControlCommand::Request& request);
    bool ParseAndSendCommand(const ArmProtocol::ArmCommandMessage& message);
    void CheckResponses();

private:
    using CommandClient = rclcpp::Client<fer_msgs::srv::ControlCommand>;
    struct PendingCommand {
        CommandClient::FutureAndRequestId future;
        double sendTime;
    };

    rclcpp::Node* node_;
    CommandClient::SharedPtr controlCommandClient_;
    std::vector<PendingCommand> pendingCommands_;
};

}

#endif
