#include "fer_sim/robot_visualizer.hpp"
#include "fer_msgs/conversions.hpp"
#include "fer_msgs/fer_defines.hpp"

#include <ctrl_toolbox_internal/Futils.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <libconfig.h++>
#include <future>
#include <rclcpp/create_timer.hpp>

using std::placeholders::_1;

namespace fer {

RobotVisualizer::RobotVisualizer()
    : Node("visualizer_node")
{
    LoadConfiguration();
    InitializeROSInterfaces();
}

void RobotVisualizer::LoadConfiguration()
{
    const bool isSim = declare_parameter("isSim", false);
    set_parameter(rclcpp::Parameter("use_sim_time", isSim));
    baseFrame_ = declare_parameter("base_frame", "panda_link0");
    jointFramePrefix_ = declare_parameter("joint_frame_prefix", "panda_joint");
    controlFrame_ = declare_parameter("control_frame", "ctrl_frame");
    loopRate_ = declare_parameter("loop_rate", 100.0);
    interactiveMarkerRate_ = declare_parameter("interactive_marker_rate", 20.0);
    if (!std::isfinite(loopRate_) || loopRate_ < 1.0 || loopRate_ > 1000.0) {
        throw std::invalid_argument("Visualizer loop_rate must be in 1..1000 Hz");
    }
    if (!std::isfinite(interactiveMarkerRate_) || interactiveMarkerRate_ < 1.0
        || interactiveMarkerRate_ > 30.0) {
        throw std::invalid_argument("interactive_marker_rate must be in 1..30 Hz");
    }
    std::string defaultFile = ament_index_cpp::get_package_share_directory("fer_ctrl") + "/conf/kcl_fer.conf";
    LoadObstacleMarkers(declare_parameter("configuration_file", defaultFile));
}

void RobotVisualizer::LoadObstacleMarkers(const std::string& configurationFile)
{
    libconfig::Config configuration;
    configuration.readFile(configurationFile.c_str());
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
        Eigen::TransformationMatrix frame;
        frame.TranslationVector(-offset * normal / normal.squaredNorm());
        frame.RotationMatrix(Eigen::RotationMatrix(
            Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), normal)));
        plane.pose = PoseToMessage(frame);
        plane.scale.x = plane.scale.y = 1.0;
        plane.scale.z = 0.002;
        plane.color.r = 0.2;
        plane.color.g = 0.9;
        plane.color.b = 0.2;
        plane.color.a = 0.3;
        planeMarkers_.push_back(plane);

        visualization_msgs::msg::Marker label;
        label.header.frame_id = baseFrame_;
        label.ns = "avoidance_plane_labels";
        label.id = index;
        label.type = label.TEXT_VIEW_FACING;
        label.action = label.ADD;
        label.pose = plane.pose;
        label.pose.position.x += 0.01 * normal.normalized().x();
        label.pose.position.y += 0.01 * normal.normalized().y();
        label.pose.position.z += 0.01 * normal.normalized().z();
        label.scale.z = 0.06;
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0;
        label.text = obstacle["name"].c_str();
        planeMarkers_.push_back(label);
    }
}

void RobotVisualizer::InitializeROSInterfaces()
{
    markerPub_ = create_publisher<visualization_msgs::msg::Marker>("visualization/arm", 1);
    visualizationPub_
        = create_publisher<visualization_msgs::msg::MarkerArray>("visualization/joint_markers", 1);
    stateDataSub_ = create_subscription<fer_msgs::msg::StateData>(topicnames::state_data,
        rclcpp::SensorDataQoS().keep_last(1), std::bind(&RobotVisualizer::HandleStateData, this, _1));
    controlDataSub_ = create_subscription<fer_msgs::msg::ControlData>(topicnames::ctrl_data,
        rclcpp::QoS(1).reliable(), std::bind(&RobotVisualizer::HandleControlData, this, _1));
    controlCommandClient_ = create_client<fer_msgs::srv::ControlCommand>(topicnames::control_command);
    interactiveMarkerServer_
        = std::make_unique<interactive_markers::InteractiveMarkerServer>(
            "tool_control", this, rclcpp::QoS(100).reliable(), rclcpp::QoS(100).reliable());
    tfPublisher_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    runTimer_ = rclcpp::create_timer(this, get_clock(),
        rclcpp::Duration::from_seconds(1.0 / loopRate_), std::bind(&RobotVisualizer::Run, this));
    // RViz drains marker updates at its display cadence, independently of simulation speed.
    interactiveMarkerTimer_ = create_wall_timer(std::chrono::duration<double>(1.0 / interactiveMarkerRate_),
        std::bind(&RobotVisualizer::UpdateToolControlMarker, this));
}

void RobotVisualizer::HandleStateData(const fer_msgs::msg::StateData& message)
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
        if (!toolFrame_.allFinite()) {
            throw std::invalid_argument("Invalid measured tool pose");
        }
        model_.JointsPosition(jointPosition);
        if (interactiveMarkerServer_->empty()) {
            CreateToolControlMarker(message.tool_pose);
        }
        lastFeedbackTime_ = now().seconds();
        receivingFeedback_ = true;
    } catch (const std::exception& exception) {
        receivingFeedback_ = false;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", exception.what());
    }
}

void RobotVisualizer::HandleControlData(const fer_msgs::msg::ControlData& message)
{
    controlData_ = message;
}

void RobotVisualizer::Run()
{
    const bool feedbackFresh = receivingFeedback_ && now().seconds() - lastFeedbackTime_ <= 0.2;
    ProcessToolCommand(feedbackFresh);
    if (!feedbackFresh) {
        return;
    }
    PublishMarkers();
}

void RobotVisualizer::UpdateToolControlMarker()
{
    if (!receivingFeedback_ || now().seconds() - lastFeedbackTime_ > 0.2) {
        return;
    }
    if (!dragging_ && !commandQueued_ && !pendingCommand_) {
        const auto pose = controlData_.ctrl_state == "move_cartesian" ? controlData_.target_pose
                                                                      : PoseToMessage(toolFrame_);
        visualization_msgs::msg::InteractiveMarker marker;
        // setPose queues a numbered update even when the pose is unchanged.
        // Avoid flooding RViz while idle or displaying a fixed Cartesian goal.
        if (interactiveMarkerServer_->get("tool_control", marker) && marker.pose != pose) {
            interactiveMarkerServer_->setPose("tool_control", pose);
        }
    }
    interactiveMarkerServer_->applyChanges();
}

void RobotVisualizer::ProcessToolCommand(bool feedbackFresh)
{
    if (pendingCommand_) {
        if (pendingCommand_->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            auto response = pendingCommand_->get();
            if (!response->success) {
                if (response->res.rfind("Command is not allowed in state ", 0) == 0) {
                    const std::string message = std::string(tc::yellow) + response->res + tc::none;
                    RCLCPP_WARN(get_logger(), "%s", message.c_str());
                } else {
                    RCLCPP_WARN(get_logger(), "%s", response->res.c_str());
                }
            }
            pendingCommand_.reset();
        } else if (now().seconds() - commandSendTime_ > 5.0) {
            controlCommandClient_->remove_pending_request(*pendingCommand_);
            pendingCommand_.reset();
            RCLCPP_WARN(get_logger(), "Controller service timed out");
        }
    }
    if (commandQueued_ && !pendingCommand_) {
        if (!feedbackFresh) {
            commandQueued_ = false;
            RCLCPP_WARN(get_logger(), "Fresh feedback is required for tool control");
        } else if (controlCommandClient_->service_is_ready()) {
            commandQueued_ = false;
            auto request = std::make_shared<fer_msgs::srv::ControlCommand::Request>(commandRequest_);
            pendingCommand_.emplace(controlCommandClient_->async_send_request(request));
            commandSendTime_ = now().seconds();
        } else if (now().seconds() - commandSendTime_ > 5.0) {
            commandQueued_ = false;
            RCLCPP_WARN(get_logger(), "Controller service is unavailable");
        }
    }
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
    const std::string modelJointPrefix = model_.ID() + rml::FrameID::Joint;
    std::array<Eigen::TransformationMatrix, ARM_JOINTS> jointFrames;
    marker.points.emplace_back();
    for (std::size_t joint = 0; joint < ARM_JOINTS; ++joint) {
        auto& jointFrame = jointFrames[joint];
        jointFrame = model_.TransformationMatrix(modelJointPrefix + std::to_string(joint));
        geometry_msgs::msg::Point point;
        point.x = jointFrame.TranslationVector().x();
        point.y = jointFrame.TranslationVector().y();
        point.z = jointFrame.TranslationVector().z();
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
        jointMarker.pose = PoseToMessage(jointFrames[joint]);
        jointMarker.scale.x = jointMarker.scale.y = 0.075;
        jointMarker.scale.z = 0.03;
        jointMarker.color.r = jointMarker.color.g = jointMarker.color.b = 0.9;
        jointMarker.color.a = 1.0;
        jointMarker.lifetime = marker.lifetime;
        markers.markers.push_back(jointMarker);

        // Geometry stays in the base frame; TF expresses each joint in its parent's frame.
        std::string parentFrame = baseFrame_;
        Eigen::TransformationMatrix parentToJoint = jointFrames[joint];
        if (joint > 0) {
            parentFrame = jointFramePrefix_ + std::to_string(joint);
            parentToJoint = model_.TransformationMatrix(modelJointPrefix + std::to_string(joint - 1),
                modelJointPrefix + std::to_string(joint));
        }
        PublishToTF(parentToJoint, parentFrame, jointFramePrefix_ + std::to_string(joint + 1));
    }
    for (auto plane : planeMarkers_) {
        plane.header.stamp = marker.header.stamp;
        plane.lifetime = marker.lifetime;
        markers.markers.push_back(plane);
    }
    visualizationPub_->publish(markers);
}

void RobotVisualizer::PublishToTF(const Eigen::TransformationMatrix& frame, const std::string& parentFrame,
    const std::string& childFrame)
{
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = parentFrame;
    transform.child_frame_id = childFrame;
    transform.transform.translation.x = frame.TranslationVector().x();
    transform.transform.translation.y = frame.TranslationVector().y();
    transform.transform.translation.z = frame.TranslationVector().z();
    transform.transform.rotation = PoseToMessage(frame).orientation;
    tfPublisher_->sendTransform(transform);
}

void RobotVisualizer::CreateToolControlMarker(const geometry_msgs::msg::Pose& pose)
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
    interactiveMarkerServer_->insert(
        marker, std::bind(&RobotVisualizer::HandleToolControlFeedback, this, _1));
    interactiveMarkerServer_->applyChanges();
}

void RobotVisualizer::HandleToolControlFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
{
    using Feedback = visualization_msgs::msg::InteractiveMarkerFeedback;
    if (feedback->marker_name != "tool_control" || feedback->header.frame_id != baseFrame_
        || !receivingFeedback_ || now().seconds() - lastFeedbackTime_ > 0.2) {
        return;
    }
    if (feedback->event_type != Feedback::MOUSE_DOWN && feedback->event_type != Feedback::POSE_UPDATE
        && feedback->event_type != Feedback::MOUSE_UP) {
        return;
    }
    try {
        Eigen::TransformationMatrix pose = MessageToPose(feedback->pose);
        if (!pose.allFinite()) {
            throw std::invalid_argument("Invalid interactive tool goal");
        }
        if (feedback->event_type == Feedback::MOUSE_DOWN) {
            dragging_ = true;
            return;
        }
        controlFramePose_ = pose;
        rml::EulerRPY rpy = pose.RotationMatrix().ToEulerRPY();
        commandRequest_.command_type = "move_cartesian";
        commandRequest_.move_type = commandRequest_.ABSMOVE;
        commandRequest_.frame_type = commandRequest_.BASE;
        commandRequest_.target_position
            = { pose.TranslationVector().x(), pose.TranslationVector().y(), pose.TranslationVector().z() };
        commandRequest_.target_orientation = { rpy.Roll(), rpy.Pitch(), rpy.Yaw() };
        PublishToTF(controlFramePose_, baseFrame_, controlFrame_);
        dragging_ = feedback->event_type == Feedback::POSE_UPDATE;
        commandQueued_ = feedback->event_type == Feedback::MOUSE_UP;
        if (commandQueued_) {
            commandSendTime_ = now().seconds();
        }
    } catch (const std::exception& exception) {
        RCLCPP_WARN(get_logger(), "%s", exception.what());
    }
}

}
