#ifndef BME_GAZEBO_SENSORS__GOAL_BRIDGE_COMPONENT_HPP_
#define BME_GAZEBO_SENSORS__GOAL_BRIDGE_COMPONENT_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "bme_gazebo_sensors_interfaces/action/execute_target_pose.hpp"
#include "bme_gazebo_sensors_interfaces/msg/motion_ui_status.hpp"
#include "bme_gazebo_sensors/visibility_control.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace bme_gazebo_sensors
{

class BME_GAZEBO_SENSORS_PUBLIC GoalBridgeComponent : public rclcpp::Node
{
public:
  using ExecuteTargetPose = bme_gazebo_sensors_interfaces::action::ExecuteTargetPose;
  using GoalHandleExecuteTargetPose = rclcpp_action::ClientGoalHandle<ExecuteTargetPose>;

  explicit GoalBridgeComponent(const rclcpp::NodeOptions & options);

private:
  void declare_and_get_parameters();

  void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  bool validate_request(
    const geometry_msgs::msg::PoseStamped & pose,
    std::string & reason) const;

  bool transform_request_to_execution_frame(
    const geometry_msgs::msg::PoseStamped & input_pose,
    geometry_msgs::msg::PoseStamped & output_pose,
    std::string & reason) const;

  void publish_status(
    const std::string & state,
    const std::string & message,
    const geometry_msgs::msg::PoseStamped & current_pose,
    double remaining_distance,
    bool goal_active) const;

  geometry_msgs::msg::PoseStamped make_default_pose(const std::string & frame_id) const;

  static bool is_finite(double value);
  static bool is_pose_finite(const geometry_msgs::msg::Pose & pose);

  rclcpp_action::Client<ExecuteTargetPose>::SharedPtr action_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_subscription_;
  rclcpp::Publisher<bme_gazebo_sensors_interfaces::msg::MotionUiStatus>::SharedPtr status_publisher_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable std::mutex goal_mutex_;
  bool goal_active_{false};
  bool have_cached_goals_{false};
  geometry_msgs::msg::PoseStamped last_requested_goal_;
  geometry_msgs::msg::PoseStamped last_execution_goal_;

  std::string action_name_;
  std::string request_topic_;
  std::string status_topic_;
  std::string input_goal_frame_;
  std::string execution_frame_;
  double server_wait_timeout_sec_{2.0};
  double transform_timeout_sec_{0.2};
};

}  // namespace bme_gazebo_sensors

#endif  // BME_GAZEBO_SENSORS__GOAL_BRIDGE_COMPONENT_HPP_