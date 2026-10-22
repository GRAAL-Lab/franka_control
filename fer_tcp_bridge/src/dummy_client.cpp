#include "fer_tcp_bridge/armprotocol.h"
#include <boost/asio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <array>
#include <cstring>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("dummy_tcp_client_node");
    try {
        std::string host = node->declare_parameter("host", "127.0.0.1");
        int port = node->declare_parameter("port", 9999);
        std::string command = node->declare_parameter("command", "");
        int maxReports = node->declare_parameter("max_reports", 0);
        if (port < 1 || port > 65535 || maxReports < 0 || (!command.empty() && command != "stop")) {
            throw std::invalid_argument("Use a valid port, max_reports >= 0, and an empty command or stop");
        }
        boost::asio::io_context ioContext;
        boost::asio::ip::tcp::socket socket(ioContext);
        socket.connect({ boost::asio::ip::make_address(host), static_cast<uint16_t>(port) });
        if (command == "stop") {
            ArmProtocol::ArmProtocolHeader header { 0, ArmProtocol::EMERGENCY_STOP, 0 };
            boost::asio::write(socket, boost::asio::buffer(&header, sizeof(header)));
        }
        socket.non_blocking(true);
        std::vector<uint8_t> input;
        rclcpp::WallRate rate(100);
        int reports = 0;
        while (rclcpp::ok() && (maxReports == 0 || reports < maxReports)) {
            std::array<uint8_t, 1024> bytes;
            boost::system::error_code error;
            std::size_t count = socket.read_some(boost::asio::buffer(bytes), error);
            input.insert(input.end(), bytes.begin(), bytes.begin() + count);
            if (error && error != boost::asio::error::would_block && error != boost::asio::error::try_again) {
                throw boost::system::system_error(error);
            }
            while (input.size() >= ArmProtocol::HEADER_SIZE) {
                ArmProtocol::ArmProtocolHeader header;
                std::memcpy(&header, input.data(), sizeof(header));
                if (header.length > ArmProtocol::MAX_REPORT_PAYLOAD) {
                    throw std::runtime_error("Invalid report length");
                }
                std::size_t size = ArmProtocol::HEADER_SIZE + header.length;
                if (input.size() < size) {
                    break;
                }
                RCLCPP_INFO(node->get_logger(), "Report %u: %u payload bytes", header.msgId, header.length);
                input.erase(input.begin(), input.begin() + size);
                ++reports;
            }
            rclcpp::spin_some(node);
            rate.sleep();
        }
    } catch (const std::exception& exception) {
        RCLCPP_ERROR(node->get_logger(), "%s", exception.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
