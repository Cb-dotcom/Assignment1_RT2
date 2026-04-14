#include "bme_gazebo_sensors/goal_bridge_component.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp_components/register_node_macro.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace bme_gazebo_sensors
{

GoalBridgeComponent::GoalBridgeComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("goal_bridge", options)
{
  declare_and_get_parameters();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  action_client_ = rclcpp_action::create_client<ExecuteTargetPose>(this, action_name_);

  target_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    request_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&GoalBridgeComponent::target_pose_callback, this, std::placeholders::_1));

  status_publisher_ =
    this->create_publisher<bme_gazebo_sensors_interfaces::msg::MotionUiStatus>(
      status_topic_,
      10);

  RCLCPP_INFO(
    get_logger(),
    "Goal bridge ready. Subscribing to '%s', publishing status on '%s', expecting user goals in '%s', transforming to '%s', and forwarding to action '%s'.",
    request_topic_.c_str(),
    status_topic_.c_str(),
    input_goal_frame_.c_str(),
    execution_frame_.c_str(),
    action_name_.c_str());

  publish_status(
    "idle",
    "Goal bridge ready.",
    make_default_pose(execution_frame_),
    0.0,
    false);
}

void GoalBridgeComponent::declare_and_get_parameters()
{
  action_name_ = this->declare_parameter<std::string>("action_name", "execute_target_pose");
  request_topic_ = this->declare_parameter<std::string>("request_topic", "/target_pose_requests");
  status_topic_ = this->declare_parameter<std::string>("status_topic", "/motion_ui_status");
  input_goal_frame_ = this->declare_parameter<std::string>("input_goal_frame", "map");
  execution_frame_ = this->declare_parameter<std::string>("execution_frame", "odom");
  server_wait_timeout_sec_ = this->declare_parameter<double>("server_wait_timeout_sec", 2.0);
  transform_timeout_sec_ = this->declare_parameter<double>("transform_timeout_sec", 0.2);

  if (action_name_.empty()) {
    throw std::invalid_argument("action_name cannot be empty.");
  }
  if (request_topic_.empty()) {
    throw std::invalid_argument("request_topic cannot be empty.");
  }
  if (status_topic_.empty()) {
    throw std::invalid_argument("status_topic cannot be empty.");
  }
  if (input_goal_frame_.empty()) {
    throw std::invalid_argument("input_goal_frame cannot be empty.");
  }
  if (execution_frame_.empty()) {
    throw std::invalid_argument("execution_frame cannot be empty.");
  }
  if (server_wait_timeout_sec_ < 0.0) {
    throw std::invalid_argument("server_wait_timeout_sec cannot be negative.");
  }
  if (transform_timeout_sec_ < 0.0) {
    throw std::invalid_argument("transform_timeout_sec cannot be negative.");
  }
}

void GoalBridgeComponent::target_pose_callback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::string reason;
  if (!validate_request(*msg, reason)) {
    RCLCPP_WARN(get_logger(), "Ignoring target pose request: %s", reason.c_str());
    publish_status(
      "request_invalid",
      reason,
      make_default_pose(execution_frame_),
      0.0,
      false);
    return;
  }

  {
    std::scoped_lock<std::mutex> lock(goal_mutex_);
    if (goal_active_) {
      const std::string busy_message =
        "Ignoring target pose request because a goal is already active.";
      RCLCPP_WARN(get_logger(), "%s", busy_message.c_str());
      publish_status(
        "busy",
        busy_message,
        make_default_pose(execution_frame_),
        0.0,
        true);
      return;
    }
  }

  geometry_msgs::msg::PoseStamped transformed_goal;
  if (!transform_request_to_execution_frame(*msg, transformed_goal, reason)) {
    RCLCPP_WARN(get_logger(), "Failed to transform target pose request: %s", reason.c_str());
    publish_status(
      "transform_failed",
      reason,
      make_default_pose(execution_frame_),
      0.0,
      false);
    return;
  }

  if (!action_client_->wait_for_action_server(std::chrono::duration<double>(server_wait_timeout_sec_))) {
    const std::string server_message =
      "Action server '" + action_name_ + "' is not available within the configured timeout.";
    RCLCPP_WARN(get_logger(), "%s", server_message.c_str());
    publish_status(
      "server_unavailable",
      server_message,
      make_default_pose(execution_frame_),
      0.0,
      false);
    return;
  }

  ExecuteTargetPose::Goal goal;
  goal.target_pose = transformed_goal;

  {
    std::scoped_lock<std::mutex> lock(goal_mutex_);
    goal_active_ = true;
    have_cached_goals_ = true;
    last_requested_goal_ = *msg;
    last_execution_goal_ = transformed_goal;
  }

  RCLCPP_INFO(
    get_logger(),
    "Forwarding target pose request from frame '%s' to execution frame '%s'. Original position [%.3f, %.3f, %.3f], transformed position [%.3f, %.3f, %.3f].",
    msg->header.frame_id.c_str(),
    transformed_goal.header.frame_id.c_str(),
    msg->pose.position.x,
    msg->pose.position.y,
    msg->pose.position.z,
    transformed_goal.pose.position.x,
    transformed_goal.pose.position.y,
    transformed_goal.pose.position.z);

  publish_status(
    "goal_submitted",
    "Goal submitted to action server.",
    make_default_pose(execution_frame_),
    0.0,
    true);

  rclcpp_action::Client<ExecuteTargetPose>::SendGoalOptions options;

  options.goal_response_callback =
    [this](GoalHandleExecuteTargetPose::SharedPtr goal_handle)
    {
      if (!goal_handle) {
        {
          std::scoped_lock<std::mutex> lock(goal_mutex_);
          goal_active_ = false;
        }

        publish_status(
          "rejected",
          "Action server rejected the submitted goal.",
          make_default_pose(execution_frame_),
          0.0,
          false);

        RCLCPP_WARN(get_logger(), "Action server rejected the submitted goal.");
        return;
      }

      publish_status(
        "accepted",
        "Action server accepted the submitted goal.",
        make_default_pose(execution_frame_),
        0.0,
        true);

      RCLCPP_INFO(get_logger(), "Action server accepted the submitted goal.");
    };

  options.feedback_callback =
    [this](
      GoalHandleExecuteTargetPose::SharedPtr,
      const std::shared_ptr<const ExecuteTargetPose::Feedback> feedback)
    {
      const auto & pose = feedback->current_pose.pose.position;

      publish_status(
        "executing",
        "Goal is being executed.",
        feedback->current_pose,
        feedback->remaining_distance,
        true);

      RCLCPP_INFO(
        get_logger(),
        "Feedback: current pose [%.3f, %.3f, %.3f], remaining distance %.3f.",
        pose.x,
        pose.y,
        pose.z,
        feedback->remaining_distance);
    };

  options.result_callback =
    [this](const GoalHandleExecuteTargetPose::WrappedResult & result)
    {
      {
        std::scoped_lock<std::mutex> lock(goal_mutex_);
        goal_active_ = false;
      }

      const auto & final_pose = result.result->final_pose.pose.position;

      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          publish_status(
            "succeeded",
            result.result->message,
            result.result->final_pose,
            0.0,
            false);
          RCLCPP_INFO(
            get_logger(),
            "Goal succeeded. Final pose [%.3f, %.3f, %.3f]. Message: %s",
            final_pose.x,
            final_pose.y,
            final_pose.z,
            result.result->message.c_str());
          break;

        case rclcpp_action::ResultCode::ABORTED:
          publish_status(
            "failed",
            result.result->message,
            result.result->final_pose,
            0.0,
            false);
          RCLCPP_WARN(
            get_logger(),
            "Goal aborted. Final pose [%.3f, %.3f, %.3f]. Message: %s",
            final_pose.x,
            final_pose.y,
            final_pose.z,
            result.result->message.c_str());
          break;

        case rclcpp_action::ResultCode::CANCELED:
          publish_status(
            "canceled",
            result.result->message,
            result.result->final_pose,
            0.0,
            false);
          RCLCPP_WARN(
            get_logger(),
            "Goal canceled. Final pose [%.3f, %.3f, %.3f]. Message: %s",
            final_pose.x,
            final_pose.y,
            final_pose.z,
            result.result->message.c_str());
          break;

        default:
          publish_status(
            "unknown",
            result.result->message,
            result.result->final_pose,
            0.0,
            false);
          RCLCPP_WARN(
            get_logger(),
            "Goal finished with unknown result code. Final pose [%.3f, %.3f, %.3f]. Message: %s",
            final_pose.x,
            final_pose.y,
            final_pose.z,
            result.result->message.c_str());
          break;
      }
    };

  action_client_->async_send_goal(goal, options);
}

bool GoalBridgeComponent::validate_request(
  const geometry_msgs::msg::PoseStamped & pose,
  std::string & reason) const
{
  if (pose.header.frame_id.empty()) {
    reason = "request frame_id cannot be empty.";
    return false;
  }

  if (pose.header.frame_id != input_goal_frame_) {
    reason = "request frame_id must be '" + input_goal_frame_ + "'.";
    return false;
  }

  if (!is_pose_finite(pose.pose)) {
    reason = "request contains non-finite pose values.";
    return false;
  }

  const double qx = pose.pose.orientation.x;
  const double qy = pose.pose.orientation.y;
  const double qz = pose.pose.orientation.z;
  const double qw = pose.pose.orientation.w;
  const double quaternion_norm_sq = (qx * qx) + (qy * qy) + (qz * qz) + (qw * qw);

  if (quaternion_norm_sq < 1e-12) {
    reason = "request orientation quaternion has near-zero norm.";
    return false;
  }

  return true;
}

bool GoalBridgeComponent::transform_request_to_execution_frame(
  const geometry_msgs::msg::PoseStamped & input_pose,
  geometry_msgs::msg::PoseStamped & output_pose,
  std::string & reason) const
{
  if (input_pose.header.frame_id == execution_frame_) {
    output_pose = input_pose;
    return true;
  }

  try {
    output_pose = tf_buffer_->transform(
      input_pose,
      execution_frame_,
      tf2::durationFromSec(transform_timeout_sec_));
    return true;
  } catch (const tf2::TransformException & ex) {
    reason =
      "failed to transform pose from frame '" + input_pose.header.frame_id +
      "' to frame '" + execution_frame_ + "': " + ex.what();
    return false;
  }
}

void GoalBridgeComponent::publish_status(
  const std::string & state,
  const std::string & message,
  const geometry_msgs::msg::PoseStamped & current_pose,
  double remaining_distance,
  bool goal_active) const
{
  bme_gazebo_sensors_interfaces::msg::MotionUiStatus status;
  status.stamp = this->now();
  status.state = state;
  status.message = message;
  status.goal_active = goal_active;
  status.remaining_distance = remaining_distance;
  status.current_pose = current_pose;

  {
    std::scoped_lock<std::mutex> lock(goal_mutex_);
    if (have_cached_goals_) {
      status.requested_goal_pose = last_requested_goal_;
      status.execution_goal_pose = last_execution_goal_;
    } else {
      status.requested_goal_pose = make_default_pose(input_goal_frame_);
      status.execution_goal_pose = make_default_pose(execution_frame_);
    }
  }

  status_publisher_->publish(status);
}

geometry_msgs::msg::PoseStamped GoalBridgeComponent::make_default_pose(
  const std::string & frame_id) const
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = this->now();
  pose.header.frame_id = frame_id;
  pose.pose.orientation.w = 1.0;
  return pose;
}

bool GoalBridgeComponent::is_finite(double value)
{
  return std::isfinite(value);
}

bool GoalBridgeComponent::is_pose_finite(const geometry_msgs::msg::Pose & pose)
{
  return
    is_finite(pose.position.x) &&
    is_finite(pose.position.y) &&
    is_finite(pose.position.z) &&
    is_finite(pose.orientation.x) &&
    is_finite(pose.orientation.y) &&
    is_finite(pose.orientation.z) &&
    is_finite(pose.orientation.w);
}

}  // namespace bme_gazebo_sensors

RCLCPP_COMPONENTS_REGISTER_NODE(bme_gazebo_sensors::GoalBridgeComponent)