#ifndef RUNE_TRACKER_HPP__
#define RUNE_TRACKER_HPP__

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <autoaim_sys_interfaces/msg/detail/tracked_rune__struct.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/float64.hpp>
#include <string>

#include "autoaim_sys_interfaces/msg/tracked_rune.hpp"
#include "rune_sys_interfaces/msg/target.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tracker/cv_model.hpp"

namespace rune
{

class TrackerNode : public rclcpp::Node
{
  enum class TargetStatus
  {
    LOST,
    TRACK
  };

  struct Tracker
  {
    using State = Eigen::Matrix<double, cv_model::EKF::N_State, 1>;
    using Measure = Eigen::Matrix<double, cv_model::EKF::N_Measure, 1>;
    using Noise_State = Eigen::Matrix<double, cv_model::EKF::N_Measure, 1>;
    using Noise_Measure = Eigen::Matrix<double, cv_model::EKF::N_Measure, 1>;
    rclcpp::Time stamp;
    State last_state;
    TargetStatus target_status = TargetStatus::LOST;
    std::shared_ptr<cv_model::EKF> ekf = std::make_shared<cv_model::EKF>();
    int chi_square_error_count = 0;
    double last_rune_angle = 0;
  };

  struct Params
  {
    double sigma_s_x = 0.0;
    double sigma_s_y = 0.0;
    double sigma_s_z = 0.0;
    double sigma_m_x = 0.0;
    double sigma_m_alpha = 0.0;
    double sigma_m_beta = 0.0;
    double chi_square_threshold = 0.0;
  };

public:
  explicit TrackerNode(const rclcpp::NodeOptions & options);
  ~TrackerNode();

private:
  bool debug_ = false;
  const std::string sub_camera_target_topic_ = "/rune/target";
  const std::string sub_camera2odom_topic_ = "/rune/transform/camera2odom";
  const std::string pub_odom_tracker_target_ = "/rune/tracker/target";

  std::unique_ptr<Params> params_ = std::make_unique<Params>();
  std::shared_ptr<Tracker> tracker_ = std::make_shared<Tracker>();
  rclcpp::TimerBase::SharedPtr update_params_timer_{};

  using TargetMsg = rune_sys_interfaces::msg::Target;
  using TrackedTargetMsg = autoaim_sys_interfaces::msg::TrackedRune;
  using TransformStamped = geometry_msgs::msg::TransformStamped;
  message_filters::Subscriber<TargetMsg> camera_target_sub_{};
  message_filters::Subscriber<TransformStamped> camera2odom_sub_{};
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<TargetMsg, TransformStamped>;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_{};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_debug_chi_;

  rclcpp::Publisher<autoaim_sys_interfaces::msg::TrackedRune>::SharedPtr pub_tracked_target_{};
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_odom_target_debug_{};

  //params
  int angle_err_count_ = 0;
  int angle_god_count_ = 0;
  void TargetCallback(
    const TargetMsg::ConstSharedPtr & target_msg,
    const TransformStamped::ConstSharedPtr & camera2odom_transform);

  void declareParams();

  void updateParams();

  double deg2rad(const double & deg) { return deg / 180 * M_PI; }
};

}  // namespace rune

#endif  // RUNE_TRACKER_HPP__