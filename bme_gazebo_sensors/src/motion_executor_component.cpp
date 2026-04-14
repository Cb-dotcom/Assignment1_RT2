#include "bme_gazebo_sensors/motion_executor_component.hpp"

#include <cmath>
#include <exception>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace bme_gazebo_sensors
{

MotionExecutorComponent::MotionExecutorComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("motion_executor", options)
{
  declare_and_get_parameters();

  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

  odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&MotionExecutorComponent::odom_callback, this, std::placeholders::_1));

  action_server_ = rclcpp_action::create_server<ExecuteTargetPose>(
    this,
    action_name_,
    std::bind(&MotionExecutorComponent::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&MotionExecutorComponent::handle_cancel, this, std::placeholders::_1),
    std::bind(&MotionExecutorComponent::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "Motion executor action server ready on '%s' using odom topic '%s', cmd_vel topic '%s', execution frame '%s'.",
    action_name_.c_str(),
    odom_topic_.c_str(),
    cmd_vel_topic_.c_str(),
    execution_frame_.c_str());
}

void MotionExecutorComponent::declare_and_get_parameters()
{
  action_name_ = this->declare_parameter<std::string>("action_name", "execute_target_pose");
  cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odom");
  execution_frame_ = this->declare_parameter<std::string>("execution_frame", "odom");

  control_rate_hz_ = this->declare_parameter<double>("control_rate_hz", 20.0);
  max_linear_velocity_ = this->declare_parameter<double>("max_linear_velocity", 0.4);
  max_angular_velocity_ = this->declare_parameter<double>("max_angular_velocity", 1.0);
  goal_position_tolerance_ = this->declare_parameter<double>("goal_position_tolerance", 0.05);
  goal_yaw_tolerance_ = this->declare_parameter<double>("goal_yaw_tolerance", 0.05);
  linear_kp_ = this->declare_parameter<double>("linear_kp", 0.8);
  angular_kp_ = this->declare_parameter<double>("angular_kp", 1.5);
  heading_alignment_threshold_rad_ =
    this->declare_parameter<double>("heading_alignment_threshold_rad", 1.0);
  odom_wait_timeout_sec_ = this->declare_parameter<double>("odom_wait_timeout_sec", 2.0);
  max_execution_time_sec_ = this->declare_parameter<double>("max_execution_time_sec", 120.0);

  if (execution_frame_.empty()) {
    throw std::invalid_argument("execution_frame cannot be empty.");
  }
  if (control_rate_hz_ <= 0.0) {
    throw std::invalid_argument("control_rate_hz must be greater than zero.");
  }
  if (max_linear_velocity_ <= 0.0) {
    throw std::invalid_argument("max_linear_velocity must be greater than zero.");
  }
  if (max_angular_velocity_ <= 0.0) {
    throw std::invalid_argument("max_angular_velocity must be greater than zero.");
  }
  if (goal_position_tolerance_ <= 0.0) {
    throw std::invalid_argument("goal_position_tolerance must be greater than zero.");
  }
  if (goal_yaw_tolerance_ <= 0.0) {
    throw std::invalid_argument("goal_yaw_tolerance must be greater than zero.");
  }
  if (odom_wait_timeout_sec_ < 0.0) {
    throw std::invalid_argument("odom_wait_timeout_sec cannot be negative.");
  }
  if (max_execution_time_sec_ <= 0.0) {
    throw std::invalid_argument("max_execution_time_sec must be greater than zero.");
  }
}

rclcpp_action::GoalResponse MotionExecutorComponent::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const ExecuteTargetPose::Goal> goal)
{
  (void)uuid;

  std::string reason;
  if (!validate_goal_pose(goal->target_pose, reason)) {
    RCLCPP_WARN(get_logger(), "Rejecting goal: %s", reason.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  {
    std::scoped_lock<std::mutex> lock(goal_mutex_);
    if (goal_active_) {
      RCLCPP_WARN(get_logger(), "Rejecting goal because another goal is already active.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    goal_active_ = true;
  }

  RCLCPP_INFO(
    get_logger(),
    "Accepted goal in frame '%s' at position [%.3f, %.3f, %.3f].",
    goal->target_pose.header.frame_id.c_str(),
    goal->target_pose.pose.position.x,
    goal->target_pose.pose.position.y,
    goal->target_pose.pose.position.z);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionExecutorComponent::handle_cancel(
  const std::shared_ptr<GoalHandleExecuteTargetPose> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(get_logger(), "Received cancel request for active goal.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionExecutorComponent::handle_accepted(
  const std::shared_ptr<GoalHandleExecuteTargetPose> goal_handle)
{
  std::thread(
    [this, goal_handle]()
    {
      execute_goal(goal_handle);
    }).detach();
}

void MotionExecutorComponent::execute_goal(
  const std::shared_ptr<GoalHandleExecuteTargetPose> goal_handle)
{
  auto result = std::make_shared<ExecuteTargetPose::Result>();

  try {
    const auto target_pose = goal_handle->get_goal()->target_pose;
    const auto start_time = this->now();
    const auto odom_deadline = start_time + rclcpp::Duration::from_seconds(odom_wait_timeout_sec_);

    rclcpp::WallRate rate(control_rate_hz_);

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        publish_stop_command();

        geometry_msgs::msg::PoseStamped final_pose;
        if (try_get_current_pose(final_pose)) {
          result->final_pose = final_pose;
        } else {
          result->final_pose.header.frame_id = execution_frame_;
          result->final_pose.header.stamp = this->now();
        }

        result->success = false;
        result->message = "Goal canceled.";
        goal_handle->canceled(result);
        release_goal_slot();
        return;
      }

      if ((this->now() - start_time).seconds() > max_execution_time_sec_) {
        publish_stop_command();

        geometry_msgs::msg::PoseStamped final_pose;
        if (try_get_current_pose(final_pose)) {
          result->final_pose = final_pose;
        } else {
          result->final_pose.header.frame_id = execution_frame_;
          result->final_pose.header.stamp = this->now();
        }

        result->success = false;
        result->message = "Goal execution timed out.";
        goal_handle->abort(result);
        release_goal_slot();
        return;
      }

      geometry_msgs::msg::PoseStamped current_pose;
      if (!try_get_current_pose(current_pose)) {
        if (this->now() < odom_deadline) {
          rate.sleep();
          continue;
        }

        publish_stop_command();
        result->success = false;
        result->message =
          "No odometry available in the configured execution frame within the allowed timeout.";
        result->final_pose.header.frame_id = execution_frame_;
        result->final_pose.header.stamp = this->now();
        goal_handle->abort(result);
        release_goal_slot();
        return;
      }

      const double dx = target_pose.pose.position.x - current_pose.pose.position.x;
      const double dy = target_pose.pose.position.y - current_pose.pose.position.y;
      const double distance_error = std::hypot(dx, dy);

      const double current_yaw = tf2::getYaw(current_pose.pose.orientation);
      const double target_yaw = tf2::getYaw(target_pose.pose.orientation);
      const double desired_heading = std::atan2(dy, dx);
      const double heading_error = normalize_angle(desired_heading - current_yaw);
      const double goal_yaw_error = normalize_angle(target_yaw - current_yaw);

      auto feedback = std::make_shared<ExecuteTargetPose::Feedback>();
      feedback->current_pose = current_pose;
      feedback->remaining_distance = distance_error;
      goal_handle->publish_feedback(feedback);

      if (distance_error <= goal_position_tolerance_ &&
        std::abs(goal_yaw_error) <= goal_yaw_tolerance_)
      {
        publish_stop_command();
        result->success = true;
        result->final_pose = current_pose;
        result->message = "Goal reached successfully.";
        goal_handle->succeed(result);
        release_goal_slot();
        return;
      }

      double linear_command = 0.0;
      double angular_command = 0.0;

      if (distance_error > goal_position_tolerance_) {
        angular_command = clamp(
          angular_kp_ * heading_error,
          -max_angular_velocity_,
          max_angular_velocity_);

        if (std::abs(heading_error) < heading_alignment_threshold_rad_) {
          linear_command = clamp(
            linear_kp_ * distance_error,
            0.0,
            max_linear_velocity_);
        }
      } else {
        angular_command = clamp(
          angular_kp_ * goal_yaw_error,
          -max_angular_velocity_,
          max_angular_velocity_);
      }

      publish_velocity_command(linear_command, angular_command);
      rate.sleep();
    }

    publish_stop_command();
    result->success = false;
    result->message = "ROS shutdown interrupted goal execution.";
    result->final_pose.header.frame_id = execution_frame_;
    result->final_pose.header.stamp = this->now();
    goal_handle->abort(result);
    release_goal_slot();
  } catch (const std::exception & ex) {
    publish_stop_command();
    result->success = false;
    result->message = std::string("Unhandled exception during goal execution: ") + ex.what();
    result->final_pose.header.frame_id = execution_frame_;
    result->final_pose.header.stamp = this->now();
    goal_handle->abort(result);
    release_goal_slot();
  } catch (...) {
    publish_stop_command();
    result->success = false;
    result->message = "Unhandled non-standard exception during goal execution.";
    result->final_pose.header.frame_id = execution_frame_;
    result->final_pose.header.stamp = this->now();
    goal_handle->abort(result);
    release_goal_slot();
  }
}

void MotionExecutorComponent::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::scoped_lock<std::mutex> lock(odom_mutex_);
  latest_odom_ = *msg;
  have_odom_ = true;
}

bool MotionExecutorComponent::validate_goal_pose(
  const geometry_msgs::msg::PoseStamped & pose,
  std::string & reason) const
{
  if (pose.header.frame_id.empty()) {
    reason = "Goal pose frame_id cannot be empty.";
    return false;
  }

  if (pose.header.frame_id != execution_frame_) {
    reason = "Goal pose must be expressed in the execution frame '" + execution_frame_ + "'.";
    return false;
  }

  if (!is_pose_finite(pose.pose)) {
    reason = "Goal pose contains non-finite values.";
    return false;
  }

  const double qx = pose.pose.orientation.x;
  const double qy = pose.pose.orientation.y;
  const double qz = pose.pose.orientation.z;
  const double qw = pose.pose.orientation.w;
  const double quaternion_norm_sq = (qx * qx) + (qy * qy) + (qz * qz) + (qw * qw);

  if (quaternion_norm_sq < 1e-12) {
    reason = "Goal pose orientation quaternion has near-zero norm.";
    return false;
  }

  return true;
}

bool MotionExecutorComponent::try_get_current_pose(
  geometry_msgs::msg::PoseStamped & current_pose) const
{
  std::scoped_lock<std::mutex> lock(odom_mutex_);

  if (!have_odom_) {
    return false;
  }

  current_pose.header = latest_odom_.header;
  if (current_pose.header.frame_id.empty()) {
    current_pose.header.frame_id = execution_frame_;
  }
  if (current_pose.header.stamp.sec == 0 && current_pose.header.stamp.nanosec == 0) {
    current_pose.header.stamp = this->now();
  }

  current_pose.pose = latest_odom_.pose.pose;
  return true;
}

void MotionExecutorComponent::publish_velocity_command(
  double linear_x,
  double angular_z) const
{
  geometry_msgs::msg::Twist command;
  command.linear.x = linear_x;
  command.angular.z = angular_z;
  cmd_vel_publisher_->publish(command);
}

void MotionExecutorComponent::publish_stop_command() const
{
  geometry_msgs::msg::Twist command;
  cmd_vel_publisher_->publish(command);
}

void MotionExecutorComponent::release_goal_slot()
{
  std::scoped_lock<std::mutex> lock(goal_mutex_);
  goal_active_ = false;
}

bool MotionExecutorComponent::is_finite(double value)
{
  return std::isfinite(value);
}

bool MotionExecutorComponent::is_pose_finite(const geometry_msgs::msg::Pose & pose)
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

double MotionExecutorComponent::clamp(
  double value,
  double min_value,
  double max_value)
{
  return std::max(min_value, std::min(value, max_value));
}

double MotionExecutorComponent::normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace bme_gazebo_sensors

RCLCPP_COMPONENTS_REGISTER_NODE(bme_gazebo_sensors::MotionExecutorComponent)