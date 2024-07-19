/**
 * @file test_node.hpp
 * @author Bin Li (lybin1336258176@outlook.com)
 * @brief 读取给定的视频，发送到识别节点进行一系列的测试
 * @date 2023-10-07
 * @version 0.1
 *
 * @copyright Copyright (c) 2023 by Bin Li
 *
 */

#ifndef TEST_NODE_HPP_
#define TEST_NODE_HPP_

// clang-format off
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <opencv2/opencv.hpp>

#include <memory>
// clang-format on

#define USE_ABS_PATH 1

namespace rune
{
class TestNode : public rclcpp::Node
{
public:
  TestNode(const rclcpp::NodeOptions & options);
  ~TestNode();

private:
  struct Impl_;
  std::unique_ptr<Impl_> impl_;

  void Publish(void);
};
}  // namespace rm

#endif  // TEST_NODE_HPP_
