/**
 * @file recognition_node.cpp
 * @author Bin Li (lybin1336258176@outlook.com)
 * @brief 接收真实相机传的图像，进一步处理和识别，判断待击打大符，并将待击打大符位置信息传递给预测节点
 * @date 2023-10-07
 * @version 0.1
 * 
 * @copyright Copyright (c) 2023 by Bin Li
 * 
 */

#include "recognition/recognition_node.hpp"

namespace rune
{
RecognitionNode::RecognitionNode(const rclcpp::NodeOptions & options) :
rclcpp::Node("RecognitionNode", options)
{
  RCLCPP_INFO(this->get_logger(), "Rune-RecognitionNode start ...");
  declareParams();

  ros_topic_channels_->img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    ros_topic_name_->SubImgTopicName,
    rclcpp::SensorDataQoS(),
    std::bind(&RecognitionNode::ImgCallback, this, std::placeholders::_1));

  ros_topic_channels_->camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    ros_topic_name_->SubCameraInfoName,
    rclcpp::SensorDataQoS(),
    std::bind(&RecognitionNode::CameraInfoCallback, this, std::placeholders::_1));

  ros_topic_channels_->fanblade_pub_ = this->create_publisher<rune_sys_interfaces::msg::Fanblade>(
    ros_topic_name_->PubFanBladeTopicName, 10);

  using std::chrono_literals::operator""ms;
  status_timer_ =
    this->create_wall_timer(200ms, std::bind(&RecognitionNode::updateNodeStatus, this));
  const int node_status = this->declare_parameter<int>("NodeStatus", 1);
  node_status_          = (node_status == 1) ? NodeStatus::ON_ACTIVATE : NodeStatus::ON_DEACTIVATE;

  if (debug_)
  {
    RCLCPP_WARN(this->get_logger(), "Rune-Debug Mode");
    RCLCPP_INFO(this->get_logger(), "Initializing debug-pub channels ...");
    ros_topic_channels_->debug_raw_img_pub_ =
      image_transport::create_publisher(this, ros_topic_name_->PubImgTopicName_debug_raw);
    ros_topic_channels_->debug_bin_img_pub_ =
      image_transport::create_publisher(this, ros_topic_name_->PubImgTopicName_debug_bin);
    ros_topic_channels_->debug_con_img_pub_ =
      image_transport::create_publisher(this, ros_topic_name_->PubImgTopicName_debug_con);
    ros_topic_channels_->debug_rec_img_pub_ =
      image_transport::create_publisher(this, ros_topic_name_->PubImgTopicName_debug_rec);
    ros_topic_channels_->debug_pnp_x_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("/rune/debug/pnp_x", rclcpp::SensorDataQoS());
    ros_topic_channels_->debug_pnp_y_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("/rune/debug/pnp_y", rclcpp::SensorDataQoS());
    ros_topic_channels_->debug_pnp_z_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("/rune/debug/pnp_z", rclcpp::SensorDataQoS());
    if (!recorder_->open(ament_index_cpp::get_package_share_directory("rune") + "record.avi",
                         cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                         30.0,
                         cv::Size(camera_info_->image_width_, camera_info_->image_height_)))
    {
      RCLCPP_ERROR(this->get_logger(), "Recorder failed to open !");
    }
    RCLCPP_INFO(this->get_logger(), "End of Initializing debug-pub channels");
    using std::chrono_literals::operator""ms;
    time_ = this->create_wall_timer(100ms, std::bind(&RecognitionNode::updateParams, this));
  } else
  {
    RCLCPP_WARN(this->get_logger(), "Release Mode");
    time_ = this->create_wall_timer(100ms, std::bind(&RecognitionNode::updateParams, this));
  }
}

RecognitionNode::~RecognitionNode()
{
  RCLCPP_WARN(this->get_logger(), "Rune-RecognitionNode ended ...");
}

void RecognitionNode::updateNodeStatus()
{
  const int node_status = this->get_parameter("NodeStatus").as_int();
  const NodeStatus next_node_status =
    (node_status == 1) ? NodeStatus::ON_ACTIVATE : NodeStatus::ON_DEACTIVATE;
  if (node_status_ != next_node_status)
  {
    if (next_node_status == NodeStatus::ON_ACTIVATE)
    {
      ros_topic_channels_->img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        ros_topic_name_->SubImgTopicName,
        rclcpp::SensorDataQoS(),
        std::bind(&RecognitionNode::ImgCallback, this, std::placeholders::_1));

      ros_topic_channels_->camera_info_sub_ =
        this->create_subscription<sensor_msgs::msg::CameraInfo>(
          ros_topic_name_->SubCameraInfoName,
          rclcpp::SensorDataQoS(),
          std::bind(&RecognitionNode::CameraInfoCallback, this, std::placeholders::_1));

      RCLCPP_WARN(this->get_logger(), "Recognition node status is activated.");
    } else
    {
      ros_topic_channels_->img_sub_->clear_on_new_intra_process_message_callback();
      ros_topic_channels_->img_sub_.reset();
      ros_topic_channels_->camera_info_sub_->clear_on_new_intra_process_message_callback();
      ros_topic_channels_->camera_info_sub_.reset();

      RCLCPP_WARN(this->get_logger(), "Recognition node status is deactivated.");
    }
    node_status_ = next_node_status;
  }
}

void RecognitionNode::ImgCallback(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg)
{
  const auto start_img_callback = this->now();

  *img_raw_ = cv_bridge::toCvShare(img_msg, "bgr8")->image;

  if (debug_)
  {
    *img_con_ = img_raw_->clone();
    *img_rec_ = img_raw_->clone();
  }

  try
  {
    preprocess(img_raw_);
  } catch (const std::exception & e)
  {
    RCLCPP_ERROR(this->get_logger(), "Rune-Preprocess Error: %s", e.what());
  }

  try
  {
    finded_waterfall_light_ = findWaterfallLight();
  } catch (const std::exception & e)
  {
    RCLCPP_ERROR(this->get_logger(), "Rune-FindR Error: %s", e.what());
  }

  try
  {
    findMoonFrame();
  } catch (const std::exception & e)
  {
    RCLCPP_ERROR(this->get_logger(), "Rune-FindT Error: %s", e.what());
  }

  try
  {
    matched_fanblade_ = matchFanBlade();
  } catch (const std::exception & e)
  {
    RCLCPP_ERROR(this->get_logger(), "Rune-FindTargetFanBlade Error: %s", e.what());
  }

  if (matched_fanblade_)
  {
    fanblade_->time_stamp_ = img_msg->header.stamp;

    // for (int i = 0; i < 4; i++)
    // {
    //   std::clog << "3D " << i << " " << fanblade_->pnp_detection_points_3D_[i] << std::endl;
    //   std::clog << "2D " << i << " " << fanblade_->pnp_detection_points_2D_[i] << std::endl;
    // }

    /* solve pnp */
    try
    {
      solvepose_->solvePnP(fanblade_->pnp_detection_points_3D_,
                           fanblade_->pnp_detection_points_2D_,
                           fanblade_->r_vec_,
                           fanblade_->t_vec_);
    } catch (const std::exception & e)
    {
      RCLCPP_ERROR(this->get_logger(), "Rune-SolvePnp Error: %s", e.what());
      return;
    }

    /* opencv camera-frame to ros2 camera-frame */
    const cv::Mat OpenCV_2_REP103 = (cv::Mat_<double>(3, 3) << 
       0, 0, 1, 
      -1, 0, 0, 
       0,-1, 0
    );
    const cv::Mat t_vec_REP_103   = OpenCV_2_REP103 * fanblade_->t_vec_;
    const cv::Mat r_vec_REP_103   = OpenCV_2_REP103 * fanblade_->r_vec_;
    fanblade_->center_3D_.x       = t_vec_REP_103.at<double>(0) / 1.0e3;
    fanblade_->center_3D_.y       = t_vec_REP_103.at<double>(1) / 1.0e3;
    fanblade_->center_3D_.z       = t_vec_REP_103.at<double>(2) / 1.0e3;

    cv::Mat r_mat;
    cv::Rodrigues(r_vec_REP_103, r_mat);

    // std::clog << "r_mat" << r_mat << std::endl;
    // std::clog << "ori r_mat" << fanblade_->r_vec_ << std::endl;
    // std::clog << "t_mat" << t_vec_REP_103 << std::endl;
    // std::clog << "ori t_mat" << fanblade_->t_vec_ << std::endl;

    rune_sys_interfaces::msg::Fanblade fanblade_msg;
    fanblade_msg.header.stamp               = img_msg->header.stamp;
    fanblade_msg.header.frame_id            = "camera_frame";
    fanblade_msg.fanblade_center.position.x = fanblade_->center_3D_.x;
    fanblade_msg.fanblade_center.position.y = fanblade_->center_3D_.y;
    fanblade_msg.fanblade_center.position.z = fanblade_->center_3D_.z;
    Eigen::Matrix<double, 3, 1> r_center(0, rune_params_->RuneRadius, rune_params_->RHeight);
    Eigen::Matrix3d rotation_matrix{
      {r_mat.at<double>(0, 0), r_mat.at<double>(0, 1), r_mat.at<double>(0, 2)},
      {r_mat.at<double>(1, 0), r_mat.at<double>(1, 1), r_mat.at<double>(1, 2)},
      {r_mat.at<double>(2, 0), r_mat.at<double>(2, 1), r_mat.at<double>(2, 2)}
    };
    Eigen::Matrix<double, 3, 1> translation_vec(t_vec_REP_103.at<double>(0, 0),
                                                t_vec_REP_103.at<double>(1, 0),
                                                t_vec_REP_103.at<double>(2, 0));
    Eigen::Matrix<double, 3, 1> r_center_camera_frame =
      rotation_matrix * r_center + translation_vec;
    fanblade_msg.r_center.position.x = r_center_camera_frame(0) / 1.0e3;
    fanblade_msg.r_center.position.y = r_center_camera_frame(1) / 1.0e3;
    fanblade_msg.r_center.position.z = r_center_camera_frame(2) / 1.0e3;
    fanblade_msg.t_vec[0]            = t_vec_REP_103.at<double>(0, 0);
    fanblade_msg.t_vec[1]            = t_vec_REP_103.at<double>(1, 0);
    fanblade_msg.t_vec[2]            = t_vec_REP_103.at<double>(2, 0);
    tf2::Matrix3x3 tf2_rot_mat(
      r_mat.at<double>(0, 0), r_mat.at<double>(0, 1), r_mat.at<double>(0, 2),
      r_mat.at<double>(1, 0), r_mat.at<double>(1, 1), r_mat.at<double>(1, 2),
      r_mat.at<double>(2, 0), r_mat.at<double>(2, 1), r_mat.at<double>(2, 2)
      );
    tf2::Quaternion tf2_quaternion;
    tf2_rot_mat.getRotation(tf2_quaternion);
    fanblade_msg.quaternion.orientation = tf2::toMsg(tf2_quaternion);

    /* RuneRotation: 0->NONE 1->CW 2->CCW */
    if (rune_rotation_ == RuneRotationStatue::NONE) {
      return;
    } else if (last_rune_rotation_ != rune_rotation_) {
      error_rotation_count_++;
      fanblade_msg.rotation = last_rune_rotation_ == RuneRotationStatue::CW ? 1u : 2u;
      if (error_rotation_count_ > 3) {
        fanblade_msg.rotation = rune_rotation_ == RuneRotationStatue::CW ? 1u : 2u;
        last_rune_rotation_ = rune_rotation_;
        error_rotation_count_ = 0;
      }
    } else {
      error_rotation_count_ = 0;
      fanblade_msg.rotation = rune_rotation_ == RuneRotationStatue::CW ? 1u : 2u;
    }
 
    const auto end_img_callback = this->now();
    timecost_->ImgCallback_cost =
      (end_img_callback - start_img_callback).to_chrono<std::chrono::milliseconds>();

    /* send rune-fanblade msg */
    ros_topic_channels_->fanblade_pub_->publish(fanblade_msg);

    /* record last-fanblade */
    if (!fanblade_->empty_)
    {
      last_fanblade_ = fanblade_;
    }
  }

  /* debug */
  if (debug_)
  {
    if (debug_params_->raw_img_)
    {
      ros_topic_channels_->debug_raw_img_pub_.publish(
        cv_bridge::CvImage(img_msg->header, "bgr8", *img_raw_).toImageMsg());
    }
    if (debug_params_->bin_img_)
    {
      ros_topic_channels_->debug_bin_img_pub_.publish(
        cv_bridge::CvImage(img_msg->header, "mono8", *img_bin_).toImageMsg());
    }
    if (debug_params_->con_img_)
    {
      ros_topic_channels_->debug_con_img_pub_.publish(
        cv_bridge::CvImage(img_msg->header, "bgr8", *img_con_).toImageMsg());
    }
    if (debug_params_->show_pnp && matched_fanblade_)
    {
      std_msgs::msg::Float64 pnp_x_msg;
      pnp_x_msg.data = fanblade_->center_3D_.x;
      std_msgs::msg::Float64 pnp_y_msg;
      pnp_y_msg.data = fanblade_->center_3D_.y;
      std_msgs::msg::Float64 pnp_z_msg;
      pnp_z_msg.data = fanblade_->center_3D_.z;
      ros_topic_channels_->debug_pnp_x_pub_->publish(pnp_x_msg);
      ros_topic_channels_->debug_pnp_y_pub_->publish(pnp_y_msg);
      ros_topic_channels_->debug_pnp_z_pub_->publish(pnp_z_msg);
      cv::drawFrameAxes(*img_rec_,
                        camera_info_->camera_matrix_,
                        camera_info_->dist_coeffs_,
                        fanblade_->r_vec_,
                        fanblade_->t_vec_,
                        70,
                        10);
    }
    if (debug_params_->rec_img_)
    {
      cv::putText(
        *img_rec_,
        "recognition cost: " + std::to_string(timecost_->ImgCallback_cost.count()) + " ms",
        cv::Point(20, 50),
        cv::FONT_HERSHEY_TRIPLEX,
        0.7,
        cv::Scalar(255, 0, 255));
      ros_topic_channels_->debug_rec_img_pub_.publish(
        cv_bridge::CvImage(img_msg->header, "bgr8", *img_rec_).toImageMsg());
      if (debug_params_->record_rec_img_ && !img_rec_->empty())
      {
        recorder_->write(*img_rec_);
      }
    }
  }
}

void RecognitionNode::CameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg)
{
  camera_info_->camera_matrix_.at<double>(0, 0) = camera_info_msg->k[0];
  camera_info_->camera_matrix_.at<double>(0, 1) = camera_info_msg->k[1];
  camera_info_->camera_matrix_.at<double>(0, 2) = camera_info_msg->k[2];
  camera_info_->camera_matrix_.at<double>(1, 0) = camera_info_msg->k[3];
  camera_info_->camera_matrix_.at<double>(1, 1) = camera_info_msg->k[4];
  camera_info_->camera_matrix_.at<double>(1, 2) = camera_info_msg->k[5];
  camera_info_->camera_matrix_.at<double>(2, 0) = camera_info_msg->k[6];
  camera_info_->camera_matrix_.at<double>(2, 1) = camera_info_msg->k[7];
  camera_info_->camera_matrix_.at<double>(2, 2) = camera_info_msg->k[8];

  camera_info_->dist_coeffs_  = camera_info_msg->d;
  camera_info_->image_width_  = camera_info_msg->width;
  camera_info_->image_height_ = camera_info_msg->height;

  solvepose_->update(camera_info_->camera_matrix_, camera_info_->dist_coeffs_);
}

}  // namespace rune

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rune::RecognitionNode)