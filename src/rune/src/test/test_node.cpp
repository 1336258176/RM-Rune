/**
 * @file test_node.cpp
 * @author Bin Li (lybin1336258176@outlook.com)
 * @brief 读取给定的视频，发送到识别节点进行一系列的测试
 * @date 2023-10-07
 * @version 0.1
 * 
 * @copyright Copyright (c) 2023 by Bin Li
 * 
 */

#include "test/test_node.hpp"

namespace rune
{
struct TestNode::Impl_
{
  const std::string kTestVideo       = "zhenghe-1.mp4";
  const std::string kPubImgTopicName = "/camera/front/capture/rune";
  const int kTransSpeed              = 1000 / 20;

#if USE_ABS_PATH
  const std::string kTestVideoPath = "/home/lybin/RoboMaster/video/" + kTestVideo;
#else
  const std::string kPkgName         = "rune";
  const std::string kPkgShareDirPath = ament_index_cpp::get_package_share_directory(kPkgName);
  const std::string kTestVideoPath   = kPkgShareDirPath + "/video/" + kTestVideo;
#endif

  cv::VideoCapture video_cap_;
  image_transport::Publisher img_puber_;
};

TestNode::TestNode(const rclcpp::NodeOptions & options) : Node("TestNode", options), impl_(std::make_unique<Impl_>())
{
  RCLCPP_INFO(this->get_logger(), "Starting TestNode ...");

  impl_->img_puber_ = image_transport::create_publisher(this, impl_->kPubImgTopicName);
  Publish();
}

TestNode::~TestNode()
{
  RCLCPP_INFO(this->get_logger(), "Ending TestNode ...");
  impl_->video_cap_.release();
}

void TestNode::Publish(void)
{
  impl_->video_cap_.open(impl_->kTestVideoPath);
  if (!impl_->video_cap_.isOpened()) {
    RCLCPP_FATAL(
      this->get_logger(), "Failed to open test-video: %s", impl_->kTestVideoPath.c_str());
    rclcpp::shutdown();
  }

  cv::Mat frame;
  while (1) {
    if (!impl_->video_cap_.read(frame)) {
      RCLCPP_INFO(this->get_logger(), "Video transmission completed, restarting...");
      impl_->video_cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
    }

    impl_->img_puber_.publish(
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg());
    cv::waitKey(impl_->kTransSpeed);
  }
}
}  // namespace rm

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rune::TestNode)