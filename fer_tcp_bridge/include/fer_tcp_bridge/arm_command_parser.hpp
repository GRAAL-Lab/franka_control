/** @file arm_command_parser.hpp */

#ifndef FER_TCP_BRIDGE_ARM_COMMAND_PARSER_HPP
#define FER_TCP_BRIDGE_ARM_COMMAND_PARSER_HPP

#include "fer_tcp_bridge/armprotocol.h"
#include "fer_msgs/srv/control_command.hpp"
#include <rclcpp/rclcpp.hpp>

namespace fer {

/** @brief Validate binary ArmProtocol commands and forward them asynchronously to the ROS controller. */
class ArmCommandParser {
public:
    /**
     * @brief Bind a command parser and client to a node.
     * @param node Owning node, which must outlive this parser; its clock times service requests.
     */
    explicit ArmCommandParser(rclcpp::Node* node);

    /**
     * @brief Translate a binary command to a validated ROS request.
     * @param message Complete ArmProtocol command, including its payload length.
     * @param[out] request Reset and populated during parsing; use only when true is returned.
     * @return False for unsupported identifiers, lengths, frames, indices or nonfinite values.
     */
    static bool ParseCommand(
        const ArmProtocol::ArmCommandMessage& message, fer_msgs::srv::ControlCommand::Request& request);
    /**
     * @brief Parse a command and queue its ROS service request without blocking.
     * @param message Complete binary command to forward.
     * @return True when queued; false on invalid input, unavailable service or a full queue.
     */
    bool ParseAndSendCommand(const ArmProtocol::ArmCommandMessage& message);
    /** @brief Collect completed service replies and discard requests older than five node-clock seconds. */
    void CheckResponses();

private:
    using CommandClient = rclcpp::Client<fer_msgs::srv::ControlCommand>;
    /** @brief Outstanding service future with its send time in node-clock seconds. */
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
