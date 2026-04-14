#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cctype>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{

std::string trim(const std::string & input)
{
  const auto first = std::find_if_not(input.begin(), input.end(), [](unsigned char c) {
    return std::isspace(c) != 0;
  });

  if (first == input.end()) {
    return "";
  }

  const auto last = std::find_if_not(input.rbegin(), input.rend(), [](unsigned char c) {
    return std::isspace(c) != 0;
  }).base();

  return std::string(first, last);
}

bool equals_ignore_case(const std::string & lhs, const std::string & rhs)
{
  if (lhs.size() != rhs.size()) {
    return false;
  }

  for (std::size_t i = 0; i < lhs.size(); ++i) {
    if (std::tolower(static_cast<unsigned char>(lhs[i])) !=
      std::tolower(static_cast<unsigned char>(rhs[i])))
    {
      return false;
    }
  }

  return true;
}

bool parse_double(const std::string & input, double & value)
{
  std::istringstream stream(input);
  stream >> value;

  if (!stream) {
    return false;
  }

  stream >> std::ws;
  return stream.eof();
}

}  // namespace

class PoseInputNode : public rclcpp::Node
{
public:
  PoseInputNode()
  : rclcpp::Node("pose_input_node")
  {
    request_topic_ = this->declare_parameter<std::string>("request_topic", "/target_pose_requests");
    frame_id_ = this->declare_parameter<std::string>("frame_id", "map");
    yaw_unit_ = this->declare_parameter<std::string>("yaw_unit", "rad");

    if (request_topic_.empty()) {
      throw std::invalid_argument("request_topic cannot be empty.");
    }

    if (frame_id_.empty()) {
      throw std::invalid_argument("frame_id cannot be empty.");
    }

    if (yaw_unit_ != "rad" && yaw_unit_ != "deg") {
      throw std::invalid_argument("yaw_unit must be either 'rad' or 'deg'.");
    }

    request_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(request_topic_, 10);

    RCLCPP_INFO(
      this->get_logger(),
      "Pose input node ready. Publishing requests to '%s' using frame '%s' and yaw unit '%s'.",
      request_topic_.c_str(),
      frame_id_.c_str(),
      yaw_unit_.c_str());
  }

  void run()
  {
    print_banner();

    while (rclcpp::ok()) {
      if (!wait_for_subscription()) {
        return;
      }

      const auto x_value = prompt_for_value("target x in world frame");
      if (!x_value.has_value()) {
        return;
      }

      const auto y_value = prompt_for_value("target y in world frame");
      if (!y_value.has_value()) {
        return;
      }

      const auto yaw_value = prompt_for_value(
        yaw_unit_ == "deg" ? "target yaw in world frame (deg)" : "target yaw in world frame (rad)");
      if (!yaw_value.has_value()) {
        return;
      }

      double yaw = *yaw_value;
      if (yaw_unit_ == "deg") {
        yaw = yaw * M_PI / 180.0;
      }

      publish_pose(*x_value, *y_value, yaw);
      print_post_publish_help();
    }
  }

private:
  std::optional<double> prompt_for_value(const std::string & label)
  {
    while (rclcpp::ok()) {
      std::cout << "Enter " << label << " (or 'q' to quit): " << std::flush;

      std::string line;
      if (!std::getline(std::cin, line)) {
        RCLCPP_INFO(this->get_logger(), "Input stream closed. Exiting UI node.");
        return std::nullopt;
      }

      line = trim(line);

      if (equals_ignore_case(line, "q") || equals_ignore_case(line, "quit") ||
        equals_ignore_case(line, "exit"))
      {
        RCLCPP_INFO(this->get_logger(), "Exit requested by user.");
        return std::nullopt;
      }

      double value = 0.0;
      if (parse_double(line, value)) {
        return value;
      }

      std::cout << "Invalid numeric input. Please try again." << std::endl;
    }

    return std::nullopt;
  }

  bool wait_for_subscription()
  {
    while (rclcpp::ok()) {
      const auto subscription_count = request_publisher_->get_subscription_count();
      if (subscription_count > 0U) {
        return true;
      }

      std::cout
        << "Waiting for a subscriber on " << request_topic_
        << " before sending requests..." << std::endl;

      rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    return false;
  }

  void publish_pose(double x, double y, double yaw_rad)
  {
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw_rad);
    quaternion.normalize();

    geometry_msgs::msg::PoseStamped request;
    request.header.stamp = this->now();
    request.header.frame_id = frame_id_;
    request.pose.position.x = x;
    request.pose.position.y = y;
    request.pose.position.z = 0.0;
    request.pose.orientation = tf2::toMsg(quaternion);

    request_publisher_->publish(request);

    RCLCPP_INFO(
      this->get_logger(),
      "Published target pose request in frame '%s': x=%.3f, y=%.3f, yaw=%.3f rad.",
      frame_id_.c_str(),
      x,
      y,
      yaw_rad);
  }

  void print_banner() const
  {
    std::cout << std::endl;
    std::cout << "==============================================" << std::endl;
    std::cout << " Target Pose Input" << std::endl;
    std::cout << "==============================================" << std::endl;
    std::cout << "Input frame: " << frame_id_ << std::endl;
    std::cout << "The target pose is interpreted in the fixed world frame." << std::endl;
    std::cout << "Yaw unit: " << yaw_unit_ << std::endl;
    std::cout << "Type 'q' at any prompt to exit." << std::endl;
    std::cout << std::endl;
  }

  void print_post_publish_help() const
  {
    std::cout << std::endl;
    std::cout << "Request published successfully." << std::endl;
    std::cout << "The goal bridge will transform the target into the robot execution frame." << std::endl;
    std::cout << "You can enter another pose." << std::endl;
    std::cout << std::endl;
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr request_publisher_;
  std::string request_topic_;
  std::string frame_id_;
  std::string yaw_unit_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PoseInputNode>();
  std::atomic_bool running{true};

  std::thread spinner([&node, &running]() {
    rclcpp::spin(node);
    running = false;
  });

  try {
    node->run();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(node->get_logger(), "Pose input node failed: %s", ex.what());
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  if (spinner.joinable()) {
    spinner.join();
  }

  return 0;
}