#include "fer_tcp_bridge/franka_tcp_bridge.hpp"
#include "fer_msgs/conversions.hpp"
#include "fer_msgs/fer_defines.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <rclcpp/create_timer.hpp>

using boost::asio::ip::tcp;
using std::placeholders::_1;

namespace fer {
namespace {

// Bound work and memory per client so socket polling cannot monopolize the ROS executor.
constexpr int MAX_COMMANDS_PER_CYCLE = 8;
constexpr std::size_t RECEIVE_CHUNK_SIZE = 512;
constexpr std::size_t MAX_RECEIVE_BUFFER_SIZE = 4096;
constexpr std::size_t MAX_SEND_BUFFER_SIZE = 65536;

}

FrankaTCPBridge::FrankaTCPBridge()
    : Node("tcp_bridge_node")
    , acceptor_(ioContext_)
    , socket_(ioContext_)
    , armCommandParser_(this)
{
    InitializeROSInterfaces();
}

void FrankaTCPBridge::InitializeROSInterfaces()
{
    const bool isSim = declare_parameter("isSim", false);
    set_parameter(rclcpp::Parameter("use_sim_time", isSim));
    std::string host = declare_parameter("host", "127.0.0.1");
    int port = declare_parameter("port", 9999);
    if (port < 1 || port > 65535) {
        throw std::invalid_argument("TCP port must be in 1..65535");
    }
    tcp::endpoint endpoint(boost::asio::ip::make_address(host), static_cast<uint16_t>(port));
    acceptor_.open(endpoint.protocol());
    acceptor_.set_option(tcp::acceptor::reuse_address(true));
    acceptor_.bind(endpoint);
    acceptor_.listen();
    acceptor_.non_blocking(true);

    stateDataSub_ = create_subscription<fer_msgs::msg::StateData>(topicnames::state_data,
        rclcpp::SensorDataQoS().keep_last(1), std::bind(&FrankaTCPBridge::HandleStateData, this, _1));
    controlDataSub_ = create_subscription<fer_msgs::msg::ControlData>(topicnames::ctrl_data,
        rclcpp::QoS(1).reliable(), std::bind(&FrankaTCPBridge::HandleControlData, this, _1));
    gripperStateSub_ = create_subscription<fer_msgs::msg::GripperState>(topicnames::gripper_state,
        rclcpp::QoS(1).reliable(), std::bind(&FrankaTCPBridge::HandleGripperState, this, _1));
    commandsTimer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(0.01),
        std::bind(&FrankaTCPBridge::CommandLoop, this));
    feedbackTimer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(0.01),
        std::bind(&FrankaTCPBridge::FeedbackLoop, this));
    RCLCPP_INFO(get_logger(), "[TCP Bridge] Binary ArmProtocol listening on %s:%d", host.c_str(), port);
}

void FrankaTCPBridge::CommandLoop()
{
    armCommandParser_.CheckResponses();
    if (!connected_) {
        boost::system::error_code error;
        acceptor_.accept(socket_, error);
        if (error == boost::asio::error::would_block || error == boost::asio::error::try_again) {
            return;
        }
        if (error) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "[TCP Bridge] Accept: %s", error.message().c_str());
            return;
        }
        socket_.non_blocking(true);
        connected_ = true;
        newFeedback_ = newControl_ = newGripper_ = true;
        RCLCPP_INFO(get_logger(), "[TCP Bridge] Client connected");
    }
    for (int count = 0; count < MAX_COMMANDS_PER_CYCLE && connected_ && ReadCommandFrame(); ++count) {
        armCommandParser_.ParseAndSendCommand(commandMessage_);
    }
    SendQueuedReports();
}

bool FrankaTCPBridge::ReadCommandFrame()
{
    std::array<uint8_t, RECEIVE_CHUNK_SIZE> input;
    boost::system::error_code error;
    std::size_t count = socket_.read_some(boost::asio::buffer(input), error);
    receiveBuffer_.insert(receiveBuffer_.end(), input.begin(), input.begin() + count);
    if (error && error != boost::asio::error::would_block && error != boost::asio::error::try_again) {
        Disconnect();
        return false;
    }
    if (receiveBuffer_.size() > MAX_RECEIVE_BUFFER_SIZE) {
        Disconnect();
        return false;
    }
    if (receiveBuffer_.size() < ArmProtocol::HEADER_SIZE) {
        return false;
    }
    ArmProtocol::ArmProtocolHeader header;
    std::memcpy(&header, receiveBuffer_.data(), sizeof(header));
    if (header.length > ArmProtocol::MAX_COMMAND_PAYLOAD) {
        RCLCPP_WARN(get_logger(), "[TCP Bridge] Invalid payload length: %u", header.length);
        Disconnect();
        return false;
    }
    std::size_t frameSize = ArmProtocol::HEADER_SIZE + header.length;
    if (receiveBuffer_.size() < frameSize) {
        return false;
    }
    commandMessage_ = {};
    std::memcpy(&commandMessage_, receiveBuffer_.data(), frameSize);
    receiveBuffer_.erase(receiveBuffer_.begin(), receiveBuffer_.begin() + frameSize);
    return true;
}

void FrankaTCPBridge::FeedbackLoop()
{
    if (!connected_) {
        return;
    }
    if (newControl_) {
        QueueControlStatusReport();
        newControl_ = false;
    }
    if (newFeedback_ && stateData_.joint_state.position.size() == ARM_JOINTS
        && stateData_.joint_state.effort.size() == ARM_JOINTS) {
        QueueRobotReports();
        newFeedback_ = false;
    }
    if (newGripper_) {
        QueueGripperReport();
        newGripper_ = false;
    }
    SendQueuedReports();
}

void FrankaTCPBridge::QueueControlStatusReport()
{
    using namespace ArmProtocol;
    ArmReportMessage report {};
    report.header = { 0, ARM_STATUS, sizeof(ArmStatus) - HEADER_SIZE };
    static const std::map<std::string, ArmStatusEnum> states = { { "disabled", DISABLED },
        { "enabling", ENABLING }, { "idle", IDLE }, { "unparking", UNPARKING }, { "parking", PARKING },
        { "parked", PARKED }, { "move_joints_pos", MOVING_JOINTS }, { "move_joints_vel", MOVING_JOINTS },
        { "move_cartesian", MOVING_EE }, { "fault", FAULT } };
    auto state = states.find(controlData_.ctrl_state);
    report.armStatus.status.code = state == states.end() ? DISABLED : state->second;
    QueueReport(report);
}

void FrankaTCPBridge::QueueRobotReports()
{
    using namespace ArmProtocol;
    ArmReportMessage report {};
    report.header = { 0, JOINTS_POSITION, sizeof(JointsPosition) - HEADER_SIZE };
    std::array<double, ARM_JOINTS + 1> position;
    std::copy_n(stateData_.joint_state.position.begin(), ARM_JOINTS, position.begin());
    position[ARM_JOINTS] = gripperState_.max_width > 0.0
        ? 100.0 * gripperState_.width / gripperState_.max_width
        : 0.0;
    std::memcpy(reinterpret_cast<char*>(&report) + HEADER_SIZE, position.data(), report.header.length);
    QueueReport(report);

    report = {};
    report.header = { 0, MOTORS_CURRENT, sizeof(MotorsCurrent) - HEADER_SIZE };
    std::memcpy(reinterpret_cast<char*>(&report) + HEADER_SIZE, stateData_.joint_state.effort.data(),
        ARM_JOINTS * sizeof(double));
    QueueReport(report);

    report = {};
    report.header = { 0, EFFECTOR_POSITION, sizeof(EffectorPosition) - HEADER_SIZE };
    try {
        Eigen::TransformationMatrix pose = MessageToPose(stateData_.tool_pose);
        rml::EulerRPY rpy = pose.RotationMatrix().ToEulerRPY();
        report.effectorPosition.x = pose.TranslationVector().x();
        report.effectorPosition.y = pose.TranslationVector().y();
        report.effectorPosition.z = pose.TranslationVector().z();
        report.effectorPosition.roll = rpy.Roll();
        report.effectorPosition.pitch = rpy.Pitch();
        report.effectorPosition.yaw = rpy.Yaw();
        QueueReport(report);
    } catch (const std::exception& exception) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[TCP Bridge] %s", exception.what());
    }
}

void FrankaTCPBridge::QueueGripperReport()
{
    using namespace ArmProtocol;
    ArmReportMessage report {};
    report.header = { 0, GRIPPER_LOCKED, sizeof(GripperLocked) - HEADER_SIZE };
    report.gripperLocked.isLocked = gripperState_.is_grasped;
    QueueReport(report);
}

void FrankaTCPBridge::QueueReport(const ArmProtocol::ArmReportMessage& report)
{
    if (!connected_) {
        return;
    }
    std::size_t size = ArmProtocol::HEADER_SIZE + report.header.length;
    if (size > sizeof(report) || sendBuffer_.size() + size > MAX_SEND_BUFFER_SIZE) {
        RCLCPP_WARN(get_logger(), "[TCP Bridge] Slow client or invalid report");
        Disconnect();
        return;
    }
    const auto* bytes = reinterpret_cast<const uint8_t*>(&report);
    sendBuffer_.insert(sendBuffer_.end(), bytes, bytes + size);
}

void FrankaTCPBridge::SendQueuedReports()
{
    if (!connected_ || sendBuffer_.empty()) {
        return;
    }
    boost::system::error_code error;
    sendOffset_ += socket_.write_some(
        boost::asio::buffer(sendBuffer_.data() + sendOffset_, sendBuffer_.size() - sendOffset_), error);
    if (error && error != boost::asio::error::would_block && error != boost::asio::error::try_again) {
        Disconnect();
    } else if (sendOffset_ == sendBuffer_.size()) {
        sendBuffer_.clear();
        sendOffset_ = 0;
    }
}

void FrankaTCPBridge::Disconnect()
{
    boost::system::error_code ignored;
    socket_.close(ignored);
    connected_ = false;
    receiveBuffer_.clear();
    sendBuffer_.clear();
    sendOffset_ = 0;
    RCLCPP_INFO(get_logger(), "[TCP Bridge] Client disconnected");
}

void FrankaTCPBridge::HandleStateData(const fer_msgs::msg::StateData& message)
{
    stateData_ = message;
    newFeedback_ = true;
}

void FrankaTCPBridge::HandleControlData(const fer_msgs::msg::ControlData& message)
{
    controlData_ = message;
    newControl_ = true;
}

void FrankaTCPBridge::HandleGripperState(const fer_msgs::msg::GripperState& message)
{
    gripperState_ = message;
    newGripper_ = true;
}

}
