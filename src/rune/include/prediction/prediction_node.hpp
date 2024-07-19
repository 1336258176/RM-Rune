#ifndef PREDICTION_NODE_HPP_
#define PREDICTION_NODE_HPP_

#include <message_filters/subscriber.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Eigen>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <queue>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/detail/float64__struct.hpp>
#include <std_msgs/msg/float64.hpp>
#include <string_view>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "prediction/bullet_solve.hpp"
#include "predictor-ceres.hpp"
#include "rune_sys_interfaces/msg/fanblade.hpp"
#include "rune_sys_interfaces/msg/target.hpp"

namespace rune
{
constexpr int QUEUE_LENGTH = 20;
constexpr int DW = 10;

/**
 * @brief 能量机关旋转状态 
 */
enum class RuneRotationStatue
{
  CW,
  /**< 顺时针*/ CCW,
  /**< 逆时针*/ NONE
};

class PredictionNode : public rclcpp::Node
{
public:
  PredictionNode(const rclcpp::NodeOptions & options);
  ~PredictionNode();

  void fanbladeCallback(const rune_sys_interfaces::msg::Fanblade::ConstSharedPtr & fanblade_msg);

private:
  bool debug_ = true;
  bool debug_record_data_ = false;

  std::string rune_statue_str_;
  RuneStatue rune_statue_;
  RuneRotationStatue rune_rotation_statue_;

  const double bullet_speed_ = 27.8;   /**< 弹速*/
  const double rune_radius_ = 700.0;   /**< 能量机关半径（mm）*/
  const double pcc_thre_ = 0.8;        /**< PCC阈值*/
  double bullet_fly_time_ = 0.0;       /**< 预测时间（子弹飞行时间）*/
  uint8_t error_prediction_count_ = 0; /**< 预测错误计数*/

  double angle_ = 0.0;      /**< 修正后的角度 rad*/
  double now_time_ = 0.0;   /**< 当前的时间 ms*/
  double now_angle_ = 0.0;  /**< 当前的角度 rad*/
  double last_time_ = 0.0;  /**< 上一帧的时间信息 ms*/
  double last_angle_ = 0.0; /**< 上一帧扇叶的角度 rad*/
  double pre_angle_ = 0;    /**< 预测角度增量 rad*/

  std::array<double, 3> world2camera_t_vec_;
  cv::Point3f target_pose_worldframe_;

  using AngleBuffer = std::array<double, QUEUE_LENGTH>;
  using TimeBuffer = std::array<double, QUEUE_LENGTH>;

  std::shared_ptr<std::deque<double>> angle_sequence_ = std::make_shared<std::deque<double>>();
  std::shared_ptr<std::deque<double>> time_sequence_ = std::make_shared<std::deque<double>>();
  /* 修正后的角度缓冲，要求非负且递增，单位为 rad */
  std::shared_ptr<AngleBuffer> angle_buffer_ = std::make_shared<AngleBuffer>();
  /* 修正后的时间缓冲，单位为 ms */
  std::shared_ptr<TimeBuffer> time_buffer_ = std::make_shared<TimeBuffer>();

  std::shared_ptr<BulletSolve> bullet_solve_ = std::make_shared<BulletSolve>();
  std::unique_ptr<rune::PredictorCeres<QUEUE_LENGTH>> predictor_ceres =
    std::make_unique<rune::PredictorCeres<QUEUE_LENGTH>>();
  // std::shared_ptr<rm::Predictor<QUEUE_LENGTH, DW>> predictor =
  //   std::make_shared<rm::Predictor<QUEUE_LENGTH, DW>>();

  const std::string fanblade_sub_topic_name_ = "/rune/transform/odom/fanblade";
  const std::string target_pub_topic_name_ = "/rune/target";
  const std::string angle_debug_pub_topic_name_ = "/rune/debug/angle";
  const std::string omega_debug_pub_topic_name_ = "/rune/debug/omega";
  const std::string pre_x_debug_pub_name_ = "/rune/debug/pre_x";
  const std::string pre_y_debug_pub_name_ = "/rune/debug/pre_y";
  const std::string pre_z_debug_pub_name_ = "/rune/debug/pre_z";

  rclcpp::Subscription<rune_sys_interfaces::msg::Fanblade>::SharedPtr sub_fanblade_{nullptr};

  rclcpp::Publisher<rune_sys_interfaces::msg::Target>::SharedPtr pub_target_{nullptr};
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_debug_angle_{nullptr};
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_debug_omega_{nullptr};
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_debug_pre_x_{nullptr};
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_debug_pre_y_{nullptr};
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_debug_pre_z_{nullptr};

  /**
   * @brief Determine the rotation state of the Rune, CW: clockwise, CCW: counterclockwise
   * 
   * @return RuneRotationStatue the rotation state of the Rune
   */
  RuneRotationStatue judgeRuneRotationStatue();

  /**
   * @brief The fan blade pose is inferred from the predicted Angle
   * 
   * @param target_pose target pose
   * @param angle Prediction Angle increment
   */
  void inferencePose(cv::Point3f & target_pose, const double dif_angle);

  /**
   * @brief The pose in the given coordinate system is transformed to the odom coordinate system
   * 
   * @param ori_pose Original pose
   * @param target_pose Target pose
   * @param ori_frame Original coordinate system
   * @param stamp timestamp
   */
  void FrameTransform2Odom(
    const cv::Point3d & ori_pose, cv::Point3d & target_pose, const std::string_view ori_frame,
    const rclcpp::Time & stamp);

  void reset()
  {
    angle_sequence_->clear();
    time_sequence_->clear();
    angle_buffer_.reset();
    time_buffer_.reset();
    angle_buffer_ = std::make_shared<AngleBuffer>();
    time_buffer_ = std::make_shared<TimeBuffer>();
  }

  /**
   * @brief record data
   * 
   * @param out_file output file 
   * @param data1 data-1
   * @param data2 data-2
   */
  void recordData(
    const std::string_view out_file_path, const std::array<double, QUEUE_LENGTH> & data1,
    const std::array<double, QUEUE_LENGTH> & data2)
  {
    std::ofstream out_file(out_file_path.data(), std::ios::out);
    if (!out_file.good())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to write data: %s", out_file_path.data());
      return;
    }
    for (int i = 0; i < QUEUE_LENGTH; i++)
    {
      out_file << data1[i] << "," << data2[i] << std::endl;
    }
    out_file.close();
  }
};

}  // namespace rune

#endif  // PREDICTION_NODE_HPP_
