#ifndef BME_GAZEBO_SENSORS__MOTION_EXECUTOR_COMPONENT_HPP_
#define BME_GAZEBO_SENSORS__MOTION_EXECUTOR_COMPONENT_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "bme_gazebo_sensors/action/execute_target_pose.hpp"
#include "bme_gazebo_sensors/visibility_control.h"


#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace bme_gazebo_sensors
{

class BME_GAZEBO_SENSORS_PUBLIC MotionExecutorComponent : public rclcpp::Node
{
public:
  using ExecuteTargetPose = bme_gazebo_sensors::action::ExecuteTargetPose;
  using GoalHandleExecuteTargetPose = rclcpp_action::ServerGoalHandle<ExecuteTargetPose>;

  explicit MotionExecutorComponent(const rclcpp::NodeOptions & options);

private:
  void declare_and_get_parameters();

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteTargetPose::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleExecuteTargetPose> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleExecuteTargetPose> goal_handle);

  void execute_goal(const std::shared_ptr<GoalHandleExecuteTargetPose> goal_handle);

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  bool validate_goal_pose(
    const geometry_msgs::msg::PoseStamped & pose,
    std::string & reason) const;

  bool try_get_current_pose(geometry_msgs::msg::PoseStamped & current_pose) const;

  void publish_velocity_command(double linear_x, double angular_z) const;
  void publish_stop_command() const;
  void release_goal_slot();

  static bool is_finite(double value);
  static bool is_pose_finite(const geometry_msgs::msg::Pose & pose);
  static double clamp(double value, double min_value, double max_value);
  static double normalize_angle(double angle);

  rclcpp_action::Server<ExecuteTargetPose>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

  mutable std::mutex odom_mutex_;
  nav_msgs::msg::Odometry latest_odom_;
  bool have_odom_{false};

  mutable std::mutex goal_mutex_;
  bool goal_active_{false};

  std::string action_name_;
  std::string cmd_vel_topic_;
  std::string odom_topic_;
  std::string execution_frame_;

  double control_rate_hz_{20.0};
  double max_linear_velocity_{0.4};
  double max_angular_velocity_{1.0};
  double goal_position_tolerance_{0.05};
  double goal_yaw_tolerance_{0.05};
  double linear_kp_{0.8};
  double angular_kp_{1.5};
  double heading_alignment_threshold_rad_{1.0};
  double odom_wait_timeout_sec_{2.0};
  double max_execution_time_sec_{120.0};
};

}  // namespace bme_gazebo_sensors

#endif  // BME_GAZEBO_SENSORS__MOTION_EXECUTOR_COMPONENT_HPP_