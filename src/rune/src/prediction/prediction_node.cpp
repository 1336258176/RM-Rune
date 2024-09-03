#include "prediction/prediction_node.hpp"

#include <rclcpp/node_options.hpp>
#include <rune_sys_interfaces/msg/detail/fanblade__struct.hpp>

namespace rune
{
PredictionNode::PredictionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("PredictionNode", options)
{
  RCLCPP_WARN(this->get_logger(), "Rune-PredictionNode start ...");
  RCLCPP_WARN(this->get_logger(), "Fanblades-Buffer queue length: %d", QUEUE_LENGTH);

  sub_fanblade_ = this->create_subscription<rune_sys_interfaces::msg::Fanblade>(
    fanblade_sub_topic_name_, 10,
    std::bind(&PredictionNode::fanbladeCallback, this, std::placeholders::_1));

  pub_target_ =
    this->create_publisher<rune_sys_interfaces::msg::Target>(target_pub_topic_name_, 10);

  rune_statue_str_ = this->declare_parameter<std::string>("autoaim_mode", "normal");

  if (debug_)
  {
    pub_debug_angle_ = this->create_publisher<std_msgs::msg::Float64>(
      angle_debug_pub_topic_name_, rclcpp::SensorDataQoS());
    pub_debug_omega_ = this->create_publisher<std_msgs::msg::Float64>(
      omega_debug_pub_topic_name_, rclcpp::SensorDataQoS());
    pub_debug_pre_x_ = this->create_publisher<std_msgs::msg::Float64>(
      pre_x_debug_pub_name_, rclcpp::SensorDataQoS());
    pub_debug_pre_y_ = this->create_publisher<std_msgs::msg::Float64>(
      pre_y_debug_pub_name_, rclcpp::SensorDataQoS());
    pub_debug_pre_z_ = this->create_publisher<std_msgs::msg::Float64>(
      pre_z_debug_pub_name_, rclcpp::SensorDataQoS());
  }
}

PredictionNode::~PredictionNode()
{
  RCLCPP_WARN(this->get_logger(), "Rune-PredictionNode ended ...");
}

void PredictionNode::fanbladeCallback(
  const rune_sys_interfaces::msg::Fanblade::ConstSharedPtr & fanblade_msg)
{
  if (error_prediction_count_ >= 10)
  {
    reset();
    error_prediction_count_ = 0;
    RCLCPP_WARN(this->get_logger(), "error prediction count >= 10, it will reset.");
    return;
  }

  /* -------------------------------------- Judge Rune Statue ------------------------------------- */
  try
  {
    rune_statue_str_ = this->get_parameter("autoaim_mode").as_string();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(this->get_logger(), "Get autoaim_mode error: %s", e.what());
  }
  // RCLCPP_INFO(this->get_logger(), "rune-statue: %s", rune_statue_str_.c_str());
  if (rune_statue_str_ == "normal" || rune_statue_str_ == "autoaim")
  {
    return;
  }
  else if (rune_statue_str_ == "big" || rune_statue_str_ == "small")
  {
    rune_statue_ = rune_statue_str_ == "big" ? RuneStatue::BIG : RuneStatue::SMALL;
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Autoaim Error!");
  }

  /* ---------------------------------- Deal with Angle and time ---------------------------------- */
  const cv::Point3d fanblade_center_odom{fanblade_msg->fanblade_center.position.x,
                                   fanblade_msg->fanblade_center.position.y,
                                   fanblade_msg->fanblade_center.position.z};
  const cv::Point3d r_center_odom{fanblade_msg->r_center.position.x,
                            fanblade_msg->r_center.position.y,
                            fanblade_msg->r_center.position.z};

  now_angle_ = std::atan2(fanblade_center_odom.z - r_center_odom.z,
                          fanblade_center_odom.y - r_center_odom.y);

  /* 角度以竖直向上为0，顺时针递增，最大不超过2PI */
  if (now_angle_ < 0)
  {
    now_angle_ += 2 * M_PI;
  }
  // RCLCPP_INFO(this->get_logger(), "angle: %lf", now_angle_ / M_PI * 180.0);

  const double dif_angle = now_angle_ - last_angle_;
  if (std::abs(dif_angle) < 0.1)
  {
    angle_ += dif_angle;
  }
  // RCLCPP_INFO(this->get_logger(), "dif-angle: %lf", dif_angle);

  /* ROS2 time stamp to ms */
  now_time_ = fanblade_msg->header.stamp.sec * 1.0e3 + fanblade_msg->header.stamp.nanosec * 1.0e-6;

  if (time_sequence_->size() != QUEUE_LENGTH)
  {
    time_sequence_->emplace_back(now_time_);

    if (angle_sequence_->size() == 1)
    {
      angle_sequence_->emplace_back(now_angle_);
      angle_ = now_angle_;
    }
    else
    {
      angle_sequence_->emplace_back(angle_);
    }
  }
  else
  {
    angle_sequence_->pop_front();
    time_sequence_->pop_front();
    angle_sequence_->emplace_back(angle_);
    time_sequence_->emplace_back(now_time_);

    std::copy(time_sequence_->begin(), time_sequence_->end(), time_buffer_->begin());
    std::copy(angle_sequence_->begin(), angle_sequence_->end(), angle_buffer_->begin());

    const double time_begin = time_buffer_->at(0);
    std::for_each(time_buffer_->begin(), time_buffer_->end(), [&time_begin](double & time) {
      time -= time_begin;
    });

    if (fanblade_msg->rotation == 0) {
      return;
    } else {
      rune_rotation_statue_ = fanblade_msg->rotation == 1 ? RuneRotationStatue::CW : RuneRotationStatue::CCW;
    }
  }

  /* ---------------------------------- Verify data availability ---------------------------------- */
  int zero_count = std::count(time_buffer_->begin(), time_buffer_->end(), 0.0);
  if (zero_count > 1)
  {
    RCLCPP_WARN(this->get_logger(), "Buffer has zero data, pass predict.");
    return;
  }

  last_angle_ = now_angle_;
  last_time_ = now_time_;

  /* -------------------------------------- Start predicting -------------------------------------- */
  if (time_sequence_->size() != QUEUE_LENGTH && rune_statue_ == RuneStatue::BIG)
  {
    RCLCPP_INFO(this->get_logger(), "Buffer-length isnt full, pass predict.");
    return;
  }

  std::copy(fanblade_msg->t_vec.begin(), fanblade_msg->t_vec.end(), world2camera_t_vec_.begin());

  // for (int i = 0; i < QUEUE_LENGTH; i++)
  // {
  //   RCLCPP_INFO(
  //     this->get_logger(), "time: %lf, angle: %lf", (*time_buffer_)[i], (*angle_buffer_)[i]);
  // }
  // RCLCPP_INFO(this->get_logger(), "last time: %lf", (*time_buffer_)[QUEUE_LENGTH - 1]);
  predictor_ceres->update((*angle_buffer_), (*time_buffer_));
  bullet_solve_->bulletCompensation(std::sqrt(fanblade_msg->fanblade_center.position.x * fanblade_msg->fanblade_center.position.x +
                                              fanblade_msg->fanblade_center.position.y * fanblade_msg->fanblade_center.position.y),
                                 fanblade_msg->fanblade_center.position.z,
                                0,
                                bullet_speed_,
                                bullet_fly_time_);
  bullet_fly_time_ = bullet_fly_time_ * 1.0e3; /**< ms*/
  // RCLCPP_INFO(this->get_logger(), "bullet_fly_time: %lf ms", bullet_fly_time_);
  // RCLCPP_INFO(this->get_logger(), "prediction init time: %lf", (*time_buffer_)[QUEUE_LENGTH - 1]);

  try
  {
    pre_angle_ = predictor_ceres->predict((*time_buffer_)[QUEUE_LENGTH - 1],
                                          bullet_fly_time_,
                                          rune_statue_,
                              (*angle_buffer_)[QUEUE_LENGTH - 1]);
    // RCLCPP_INFO(this->get_logger(), "init angle: %lf", (*angle_buffer_)[QUEUE_LENGTH - 1]);
    if (!std::isnormal(pre_angle_))
    {
      RCLCPP_WARN(this->get_logger(), "pre_angle_ is not normal.");
      error_prediction_count_++;
      return;
    }
    if (pre_angle_ > 60.0 / 180.0 * M_PI)
    {
      RCLCPP_WARN(this->get_logger(), "pre_angle_ is too big, it will reset!");
      error_prediction_count_++;
      return;
    }
    // RCLCPP_INFO(this->get_logger(), "pre-angle: %lf", pre_angle_ / M_PI * 180.0);
    // RCLCPP_INFO(this->get_logger(), "cost-time: %ld", predictor_ceres->getTimeCost());
    // RCLCPP_INFO(this->get_logger(), "omega: %lf", predictor_ceres->getOmega());
    // double params[5];
    // predictor_ceres->getParams(params);
    // RCLCPP_INFO(this->get_logger(),
    //             "params: %lf, %lf, %lf, %lf, %lf",
    //             params[0],
    //             params[1],
    //             params[2],
    //             params[3],
    //             params[4]);
    inferencePose(target_pose_worldframe_, pre_angle_);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(this->get_logger(), "Rune-PredictionNode Error: %s", e.what());
    return;
  }

  /* --------------------------------- world_frame to camera_frame -------------------------------- */
  const Eigen::Quaternion world2camera (fanblade_msg->quaternion.orientation.w,
                                        fanblade_msg->quaternion.orientation.x,
                                        fanblade_msg->quaternion.orientation.y,
                                        fanblade_msg->quaternion.orientation.z);
  const Eigen::Matrix3d world2camera_rotation = world2camera.toRotationMatrix();
  Eigen::Matrix<double, 3, 1> world_point {target_pose_worldframe_.x,
                                           target_pose_worldframe_.y,
                                           target_pose_worldframe_.z};
  const Eigen::Matrix<double, 3, 3> OpenCV_2_REP103 {{ 0, 0, 1},
                                                     {-1, 0, 0},
                                                     { 0,-1, 0}};
  world_point = OpenCV_2_REP103 * world_point;
  const Eigen::Matrix<double, 3, 1> t {world2camera_t_vec_[0],
                                       world2camera_t_vec_[1],
                                       world2camera_t_vec_[2]};
  Eigen::Matrix<double, 3, 1> target_pose_cameraframe = world2camera_rotation * world_point + t;

  target_pose_cameraframe[0] /= 1.0e3;
  target_pose_cameraframe[1] /= 1.0e3;
  target_pose_cameraframe[2] /= 1.0e3;

  /* ------------------------------------- building Target-msg ------------------------------------ */
  rune_sys_interfaces::msg::Target target_msg;
  target_msg.header.stamp = fanblade_msg->header.stamp;
  target_msg.header.frame_id = "camera_frame";
  target_msg.angle = now_angle_;
  target_msg.is_tracked = true;
  target_msg.pose.position.x = target_pose_cameraframe(0);
  target_msg.pose.position.y = target_pose_cameraframe(1);
  target_msg.pose.position.z = target_pose_cameraframe(2);
  pub_target_->publish(target_msg);

  /* -------------------------------------------- debug ------------------------------------------- */
  if (debug_)
  {
    std_msgs::msg::Float64 omega_msg;
    omega_msg.data = predictor_ceres->getOmega();
    pub_debug_omega_->publish(omega_msg);
    std_msgs::msg::Float64 pre_x;
    pre_x.data = target_pose_cameraframe[0];
    pub_debug_pre_x_->publish(pre_x);
    std_msgs::msg::Float64 pre_y;
    pre_y.data = target_pose_cameraframe[1];
    pub_debug_pre_y_->publish(pre_y);
    std_msgs::msg::Float64 pre_z;
    pre_z.data = target_pose_cameraframe[2];
    pub_debug_pre_z_->publish(pre_z);
    std_msgs::msg::Float64 angle_msg;
    angle_msg.data = angle_;
    pub_debug_angle_->publish(angle_msg);
    if (debug_record_data_)
    {
      recordData("./big-rune.csv", *time_buffer_, *angle_buffer_);
    }
  }
}

void PredictionNode::inferencePose(cv::Point3f & target_pose, const double dif_angle)
{
  const double angle = std::abs(dif_angle);
  double x     = rune_radius_ * std::sin(angle);
  double y     = rune_radius_ * (1 - std::cos(angle));

  if (rune_rotation_statue_ == RuneRotationStatue::CW)
  {
    x = -x;
  }

  target_pose.x = x;
  target_pose.y = y;
  target_pose.z = 0;
}

}  // namespace rune

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rune::PredictionNode)