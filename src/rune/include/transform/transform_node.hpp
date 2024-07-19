#ifndef COORDINATE_HPP_
#define COORDINATE_HPP_

// C++
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <queue>
#include <random>
#include <ranges>
#include <rclcpp/node_options.hpp>
#include <string>
#include <unordered_map>
#include <vector>

// Eigen
#include <tf2_eigen/tf2_eigen.hpp>

// ROS2 .h
#include <message_filters/subscriber.h>
#include <rcl/event.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "rune_sys_interfaces/msg/fanblade.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"

namespace rune
{

class TransformNode : public rclcpp::Node
{
public:
  TransformNode(const rclcpp::NodeOptions & options);
  ~TransformNode();

  void fanbladeCallback(const rune_sys_interfaces::msg::Fanblade::ConstSharedPtr & msg);

  void frameTransform2Odom(const double (&ori_pose)[3],
                           double (&target_pose)[3],
                           const std::string_view ori_frame,
                           const rclcpp::Time & stamp);

private:
  const std::string sub_fanblade_topic_name_ = "/rune/fanblade";
  const std::string pub_fanblade_odom_topic_ = "/rune/transform/odom/fanblade";
  const std::string pub_camera2odom_topic_   = "/rune/transform/camera2odom";

  // Publisher
  rclcpp::Publisher<rune_sys_interfaces::msg::Fanblade>::SharedPtr pub_fanblade_odom_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr pub_camera2odom_{nullptr};

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_{nullptr};
  message_filters::Subscriber<rune_sys_interfaces::msg::Fanblade> fanblade_sub_{};
  std::shared_ptr<tf2_ros::MessageFilter<rune_sys_interfaces::msg::Fanblade>> tf2_fanblade_filter_{
    nullptr};
};

}  // namespace rune

#endif  // COORDINATE_HPP_