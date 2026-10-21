#include "fer_sim/robot_visualizer.hpp"
#include "fer_msgs/conversions.hpp"
#include "fer_msgs/fer_defines.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <libconfig.h++>
#include <future>

using std::placeholders::_1;

namespace fer {

RobotVisualizer::RobotVisualizer()
    : Node("visualizer_node")
{
    LoadConfiguration();
    Initialization();
}

void RobotVisualizer::LoadConfiguration()
{
    baseFrame_ = declare_parameter("base_frame", "panda_link0");
    jointFramePrefix_ = declare_parameter("joint_frame_prefix", "panda_joint");
    controlFrame_ = declare_parameter("control_frame", "ctrl_frame");
    loopRate_ = declare_parameter("loop_rate", 100.0);
    if (!std::isfinite(loopRate_) || loopRate_ < 1.0 || loopRate_ > 1000.0) {
        throw std::invalid_argument("Visualizer loop_rate must be in 1..1000 Hz");
    }
    std::string defaultFile = ament_index_cpp::get_package_share_directory("fer_ctrl") + "/conf/kcl_fer.conf";
    libconfig::Config configuration;
    configuration.readFile(declare_parameter("configuration_file", defaultFile).c_str());
    const auto& obstacles = configuration.getRoot()["obstacles"];
    for (int index = 0; index < obstacles.getLength(); ++index) {
        const auto& obstacle = obstacles[index];
        const auto& values = obstacle["normal"];
        if (values.getLength() != 3) {
            throw std::invalid_argument("Obstacle normal must contain three values");
        }
        Eigen::Vector3d normal(values[0], values[1], values[2]);
        double offset = obstacle["offset"];
        if (!normal.allFinite() || normal.norm() < 1e-9 || !std::isfinite(offset)) {
            throw std::invalid_argument("Invalid visualizer obstacle plane");
        }
        visualization_msgs::msg::Marker plane;
        plane.header.frame_id = baseFrame_;
        plane.ns = "avoidance_planes";
        plane.id = index;
        plane.type = plane.CUBE;
        plane.action = plane.ADD;
        Pose frame = Pose::Identity();
        frame.translation() = -offset * normal / normal.squaredNorm();
        frame.linear()
            = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), normal).toRotationMatrix();
        plane.pose = PoseToMessage(frame);
        plane.scale.x = plane.scale.y = 1.0;
        plane.scale.z = 0.002;
        plane.color.r = 0.2;
        plane.color.g = 0.9;
        plane.color.b = 0.2;
        plane.color.a = 0.3;
        planeMarkers_.push_back(plane);
    }
}

void RobotVisualizer::Initialization()
{
    markerPub_ = create_publisher<visualization_msgs::msg::Marker>("visualization/arm", 1);
    visualizationPub_
        = create_publisher<visualization_msgs::msg::MarkerArray>("visualization/joint_markers", 1);
    stateDataSub_ = create_subscription<fer_msgs::msg::StateData>(topicnames::state_data,
        rclcpp::SensorDataQoS().keep_last(1), std::bind(&RobotVisualizer::StateDataCB, this, _1));
    controlDataSub_ = create_subscription<fer_msgs::msg::ControlData>(topicnames::ctrl_data,
        rclcpp::QoS(1).reliable(), std::bind(&RobotVisualizer::ControlDataCB, this, _1));
    controlCommandClient_ = create_client<fer_msgs::srv::ControlCommand>(topicnames::control_command);
    intMarkerServer_ = std::make_unique<interactive_markers::InteractiveMarkerServer>("tool_control", this);
    tfPublisher_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    runTimer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / loopRate_), std::bind(&RobotVisualizer::Run, this));
}

void RobotVisualizer::StateDataCB(const fer_msgs::msg::StateData& message)
{
    if (!message.connected || message.header.frame_id != baseFrame_
        || message.joint_state.name != JointNames() || message.joint_state.position.size() != ARM_JOINTS) {
        receivingFeedback_ = false;
        return;
    }
    JointVector jointPosition = Eigen::Map<const JointVector>(message.joint_state.position.data());
    if (!jointPosition.allFinite()) {
        receivingFeedback_ = false;
        return;
    }

    try {
        toolFrame_ = MessageToPose(message.tool_pose);
        if (!toolFrame_.matrix().allFinite()) {
            throw std::invalid_argument("Invalid measured tool pose");
        }
        kinematics_ = PandaModel().ComputeKinematics(jointPosition);
        if (intMarkerServer_->empty()) {
            make6DofMarker(message.tool_pose);
        }
        lastFeedbackTime_ = SteadyTime();
        receivingFeedback_ = true;
    } catch (const std::exception& exception) {
        receivingFeedback_ = false;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[Visualizer] %s", exception.what());
    }
}

void RobotVisualizer::ControlDataCB(const fer_msgs::msg::ControlData& message)
{
    controlDataMsg_ = message;
}

void RobotVisualizer::Run()
{
    if (pendingCommand_) {
        if (pendingCommand_->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            auto response = pendingCommand_->get();
            if (!response->success) {
                RCLCPP_WARN(get_logger(), "[Visualizer] %s", response->res.c_str());
            }
            pendingCommand_.reset();
        } else if (SteadyTime() - commandSendTime_ > 5.0) {
            controlCommandClient_->remove_pending_request(*pendingCommand_);
            pendingCommand_.reset();
            RCLCPP_WARN(get_logger(), "[Visualizer] Controller service timed out");
        }
    }
    bool feedbackFresh = receivingFeedback_ && SteadyTime() - lastFeedbackTime_ <= 0.2;
    if (sendCommandToController_ && !pendingCommand_) {
        if (!feedbackFresh) {
            sendCommandToController_ = false;
            RCLCPP_WARN(get_logger(), "[Visualizer] Fresh feedback is required for tool control");
        } else if (controlCommandClient_->service_is_ready()) {
            sendCommandToController_ = false;
            auto request = std::make_shared<fer_msgs::srv::ControlCommand::Request>(commandRequest_);
            pendingCommand_.emplace(controlCommandClient_->async_send_request(request));
            commandSendTime_ = SteadyTime();
        } else if (SteadyTime() - commandSendTime_ > 5.0) {
            sendCommandToController_ = false;
            RCLCPP_WARN(get_logger(), "[Visualizer] Controller service is unavailable");
        }
    }
    if (!feedbackFresh) {
        return;
    }
    PublishMarkers();
    if (!dragging_ && !sendCommandToController_ && !pendingCommand_) {
        auto pose = controlDataMsg_.ctrl_state == "move_cartesian" ? controlDataMsg_.target_pose
                                                                   : PoseToMessage(toolFrame_);
        intMarkerServer_->setPose("tool_control", pose);
    }
    intMarkerServer_->applyChanges();
}

void RobotVisualizer::PublishMarkers()
{
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = baseFrame_;
    marker.ns = "fer_arm";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.045;
    marker.color.r = 0.8;
    marker.color.g = 0.85;
    marker.color.b = 0.9;
    marker.color.a = 1.0;
    marker.lifetime.nanosec = 500000000;
    for (const Pose& jointFrame : kinematics_.jointFrames) {
        geometry_msgs::msg::Point point;
        point.x = jointFrame.translation().x();
        point.y = jointFrame.translation().y();
        point.z = jointFrame.translation().z();
        marker.points.push_back(point);
    }
    marker.points.push_back(PoseToMessage(toolFrame_).position);
    markerPub_->publish(marker);
    markers.markers.push_back(marker);

    visualization_msgs::msg::Marker base;
    base.header = marker.header;
    base.ns = "manipulator_base";
    base.type = base.CUBE;
    base.pose.orientation.w = 1.0;
    base.pose.position.z = 0.0125;
    base.scale.x = base.scale.y = 0.12;
    base.scale.z = 0.025;
    base.color.r = 0.8;
    base.color.g = 0.5;
    base.color.a = 1.0;
    base.lifetime = marker.lifetime;
    markers.markers.push_back(base);
    for (std::size_t joint = 0; joint < ARM_JOINTS; ++joint) {
        visualization_msgs::msg::Marker jointMarker;
        jointMarker.header = marker.header;
        jointMarker.ns = "manipulator_joints";
        jointMarker.id = static_cast<int>(joint);
        jointMarker.type = jointMarker.CYLINDER;
        jointMarker.pose = PoseToMessage(kinematics_.jointFrames[joint + 1]);
        jointMarker.scale.x = jointMarker.scale.y = 0.075;
        jointMarker.scale.z = 0.03;
        jointMarker.color.r = jointMarker.color.g = jointMarker.color.b = 0.9;
        jointMarker.color.a = 1.0;
        jointMarker.lifetime = marker.lifetime;
        markers.markers.push_back(jointMarker);
        PublishToTF(kinematics_.jointFrames[joint + 1], jointFramePrefix_ + std::to_string(joint + 1));
    }
    for (auto plane : planeMarkers_) {
        plane.header.stamp = marker.header.stamp;
        plane.lifetime = marker.lifetime;
        markers.markers.push_back(plane);
    }
    visualizationPub_->publish(markers);
}

void RobotVisualizer::PublishToTF(const Pose& frame, const std::string& childFrame)
{
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = baseFrame_;
    transform.child_frame_id = childFrame;
    transform.transform.translation.x = frame.translation().x();
    transform.transform.translation.y = frame.translation().y();
    transform.transform.translation.z = frame.translation().z();
    transform.transform.rotation = PoseToMessage(frame).orientation;
    tfPublisher_->sendTransform(transform);
}

void RobotVisualizer::make6DofMarker(const geometry_msgs::msg::Pose& pose)
{
    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = baseFrame_;
    marker.name = "tool_control";
    marker.description = "FER tool goal";
    marker.scale = 0.2;
    marker.pose = pose;
    visualization_msgs::msg::InteractiveMarkerControl boxControl;
    boxControl.name = "move_rotate_3d";
    boxControl.always_visible = true;
    boxControl.interaction_mode = boxControl.MOVE_ROTATE_3D;
    visualization_msgs::msg::Marker box;
    box.type = box.CUBE;
    box.pose.orientation.w = 1.0;
    box.scale.x = box.scale.y = box.scale.z = 0.035;
    box.color.r = 0.2;
    box.color.g = 0.6;
    box.color.b = 1.0;
    box.color.a = 0.8;
    boxControl.markers.push_back(box);
    marker.controls.push_back(boxControl);
    for (int axis = 0; axis < 3; ++axis) {
        visualization_msgs::msg::InteractiveMarkerControl control;
        Eigen::Quaterniond orientation
            = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), Eigen::Vector3d::Unit(axis));
        control.orientation.x = orientation.x();
        control.orientation.y = orientation.y();
        control.orientation.z = orientation.z();
        control.orientation.w = orientation.w();
        std::string axisName(1, "xyz"[axis]);
        control.name = "rotate_" + axisName;
        control.interaction_mode = control.ROTATE_AXIS;
        marker.controls.push_back(control);
        control.name = "move_" + axisName;
        control.interaction_mode = control.MOVE_AXIS;
        marker.controls.push_back(control);
    }
    intMarkerServer_->insert(marker, std::bind(&RobotVisualizer::interactiveMarkerCB, this, _1));
    intMarkerServer_->applyChanges();
}

void RobotVisualizer::interactiveMarkerCB(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
{
    using Feedback = visualization_msgs::msg::InteractiveMarkerFeedback;
    if (feedback->marker_name != "tool_control" || feedback->header.frame_id != baseFrame_
        || !receivingFeedback_ || SteadyTime() - lastFeedbackTime_ > 0.2) {
        return;
    }
    if (feedback->event_type != Feedback::MOUSE_DOWN && feedback->event_type != Feedback::POSE_UPDATE
        && feedback->event_type != Feedback::MOUSE_UP) {
        return;
    }
    try {
        Pose pose = MessageToPose(feedback->pose);
        if (!pose.matrix().allFinite()) {
            throw std::invalid_argument("Invalid interactive tool goal");
        }
        if (feedback->event_type == Feedback::MOUSE_DOWN) {
            dragging_ = true;
            return;
        }
        controlFramePose_ = pose;
        auto rpy = Eigen::RotationMatrix(pose.linear()).ToEulerRPY();
        commandRequest_.command_type = "move_cartesian";
        commandRequest_.move_type = commandRequest_.ABSMOVE;
        commandRequest_.frame_type = commandRequest_.BASE;
        commandRequest_.target_position
            = { pose.translation().x(), pose.translation().y(), pose.translation().z() };
        commandRequest_.target_orientation = { rpy.Roll(), rpy.Pitch(), rpy.Yaw() };
        PublishToTF(controlFramePose_, controlFrame_);
        dragging_ = feedback->event_type == Feedback::POSE_UPDATE;
        sendCommandToController_ = feedback->event_type == Feedback::MOUSE_UP;
        if (sendCommandToController_) {
            commandSendTime_ = SteadyTime();
        }
    } catch (const std::exception& exception) {
        RCLCPP_WARN(get_logger(), "[Visualizer] %s", exception.what());
    }
}

}
