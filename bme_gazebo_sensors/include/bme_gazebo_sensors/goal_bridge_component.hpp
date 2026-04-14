#ifndef BME_GAZEBO_SENSORS__GOAL_BRIDGE_COMPONENT_HPP_
#define BME_GAZEBO_SENSORS__GOAL_BRIDGE_COMPONENT_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "bme_gazebo_sensors/action/execute_target_pose.hpp"
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
  using ExecuteTargetPose = bme_gazebo_sensors::action::ExecuteTargetPose;
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

  static bool is_finite(double value);
  static bool is_pose_finite(const geometry_msgs::msg::Pose & pose);

  rclcpp_action::Client<ExecuteTargetPose>::SharedPtr action_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_subscription_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable std::mutex goal_mutex_;
  bool goal_active_{false};

  std::string action_name_;
  std::string request_topic_;
  std::string input_goal_frame_;
  std::string execution_frame_;
  double server_wait_timeout_sec_{2.0};
  double transform_timeout_sec_{0.2};
};

}  // namespace bme_gazebo_sensors

#endif  // BME_GAZEBO_SENSORS__GOAL_BRIDGE_COMPONENT_HPP_