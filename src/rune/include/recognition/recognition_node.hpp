/**
 * @file recognition_node.hpp
 * @author Bin Li (lybin1336258176@outlook.com)
 * @brief 接收真实相机传的图像，进一步处理和识别，判断待击打大符，并将待击打大符位置信息传递给预测节点
 * @date 2023-10-07
 * @version 0.1
 * 
 * @copyright Copyright (c) 2023 by Bin Li
 * 
 */

#ifndef RECOGNITION_NODE_HPP_
#define RECOGNITION_NODE_HPP_

#include <cv_bridge/cv_bridge.h>
#include <Eigen/Eigen>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rune_sys_interfaces/msg/fanblade.hpp"
#include "solvepose.hpp"

#include <opencv2/opencv.hpp>

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

namespace rune
{
class RecognitionNode : public rclcpp::Node
{
public:
  RecognitionNode(const rclcpp::NodeOptions & options);
  ~RecognitionNode();

  /**
   * @brief 节点状态
   */
  enum class NodeStatus {
    ON_DEACTIVATE = 0, /**< 停用*/
    ON_ACTIVATE   = 1  /**< 激活*/
  };

  /**
   * @brief Moon框状态
   */
  enum class Statue {
    NO_MATCHING, /**< 未匹配*/
    MATCHING     /**< 已匹配*/
  };

  /**
   * @brief 能量机关旋转状态 
   */
  enum class RuneRotationStatue
  {
    CW,   /**< 顺时针*/
    CCW,  /**< 逆时针*/
    NONE
  };

  /**
   * @brief 判断当前帧的扇叶与上一帧的扇叶是否接近重合
   */
  enum class FanbladeStatue {
    NO_MATCHING, /**< 未匹配*/
    MATCHING     /**< 已匹配*/
  };

  enum class LightColor {
    RED  = 1, /**< 红色*/
    BLUE = 2  /**< 蓝色*/
  };

  struct RosTopicName
  {
    std::string SubImgTopicName           = "/camera/front/capture";     /**< 接收的原图*/
    std::string SubCameraInfoName         = "/camera/front/camera_info"; /**< 接收相机信息*/
    std::string PubFanBladeTopicName      = "/rune/fanblade";      /**< 发布识别到的扇叶*/
    std::string PubImgTopicName_debug_raw = "/rune/debug/raw_img"; /**< 接收的原图*/
    std::string PubImgTopicName_debug_bin = "/rune/debug/bin_img"; /**< 二值化图*/
    std::string PubImgTopicName_debug_con = "/rune/debug/con_img"; /**< 轮廓图*/
    std::string PubImgTopicName_debug_rec = "/rune/debug/rec_img"; /**< 识别图*/
  };

  struct RosTopicChannels
  {
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_{nullptr};
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_{nullptr};
    rclcpp::Publisher<rune_sys_interfaces::msg::Fanblade>::SharedPtr fanblade_pub_{nullptr};
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_pnp_x_pub_{nullptr};
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_pnp_y_pub_{nullptr};
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_pnp_z_pub_{nullptr};
    image_transport::Publisher debug_raw_img_pub_;
    image_transport::Publisher debug_bin_img_pub_;
    image_transport::Publisher debug_con_img_pub_;
    image_transport::Publisher debug_rec_img_pub_;

    ~RosTopicChannels()
    {
      debug_raw_img_pub_.shutdown();
      debug_bin_img_pub_.shutdown();
      debug_con_img_pub_.shutdown();
      debug_rec_img_pub_.shutdown();
    }
  };

  struct WaterfallLight
  {
    float radius_;
    double aspect_ratio_;
    cv::Rect2f R_Rect_;
    cv::Point2f center_;
  
    WaterfallLight (float radius, double aspect_ratio, cv::Rect2f R_Rect, cv::Point2f center) :
                   radius_(radius), aspect_ratio_(aspect_ratio), R_Rect_(R_Rect), center_(center)
    {
    }
    WaterfallLight () = default;
    ~WaterfallLight () = default;
  };

  struct MoonFrame
  {
    double aspect_ratio_;
    Statue statue_ = Statue::NO_MATCHING;
    cv::Point2f center_;
    std::array<cv::Point2f, 4> points_;
    cv::RotatedRect rotated_rect_;

    MoonFrame (double aspect_ratio, Statue statue, cv::Point2f center, std::array<cv::Point2f, 4> points, cv::RotatedRect rotated_rect) :
              aspect_ratio_(aspect_ratio), statue_(statue), center_(center), points_(points), rotated_rect_(rotated_rect)
    {
    }
    MoonFrame () = default;
    ~MoonFrame () = default;
  };

  struct FanBlade
  {
    bool empty_    = true;
    cv::Point2f center_;    /**< Two-dimensional pixel coordinates*/
    cv::Point3f center_3D_; /**< Three-dimensional world coordinates in ros2 camera-frame*/
    cv::Mat t_vec_;         /**< opencv solvepnp*/
    cv::Mat r_vec_;         /**< opencv solvepnp*/
    MoonFrame outside_frame_;
    MoonFrame inside_frame_;
    rclcpp::Time time_stamp_;
    FanbladeStatue fanblade_statue_ = FanbladeStatue::NO_MATCHING;
    std::array<cv::Point2f, 4> pnp_detection_points_2D_;
    std::array<cv::Point3f, 4> pnp_detection_points_3D_;
  };

  struct RuneParams
  {
    std::array<double, 7> R_Hu_min{0, -0.05, -0.05, -0.05, -0.05, -0.05, -0.05};
    std::array<double, 7> R_Hu_max{0.5, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05};
    std::array<double, 2> T_Hu_min{0.2, 0};
    std::array<double, 2> T_Hu_max{1, 0.5};
    std::array<double, 2> Moon_Hu_min{0.75, 0.4};
    std::array<double, 2> Moon_Hu_max{2, 3};
    double BinaryThres_Blue        = 0; /**< 适用蓝色能量机关二值化阈值*/
    double BinartThres_Red         = 0; /**< 适用红色能量机关二值化阈值*/
    double BinaryThres             = 0; /**< 平均二值化阈值*/
    int    MinWaterfallLightCount  = 0; /**< 最小流水灯数量*/
    double RuneRadius              = 0; /**< 能量机关半径（圆心到扇叶中心 mm）*/
    double RHeight                 = 0; /**< R型标高度*/
    double WaterfallLightMaxDistanceDiff = 0; /**< 流水灯最大距离差*/
    double WaterfallLightMinArea   = 0; /**< 流水灯最小面积*/
    double WaterfallLightMaxArea   = 0; /**< 流水灯最大面积*/
    double WaterfallLightMinROF    = 0; /**< 流水灯最小填充率*/
    double WaterfallLightMaxROF    = 0; /**< 流水灯最大填充率*/
    double WaterfallLightMinAspectRatio = 0; /**< 流水灯最小宽高比*/
    double WaterfallLightMaxAspectRatio = 0; /**< 流水灯最大宽高比*/
    double MoonFrameMinArea        = 0; /**< 半月框最小面积*/
    double MoonFrameMaxArea        = 0; /**< 半月框最大面积*/
    double MoonFrameMinAspectRatio = 0; /**< 半月框最小宽高比*/
    double MoonFrameMaxAspectRatio = 0; /**< 半月框最大宽高比*/
    double MoonFrameMinROF         = 0; /**< 半月框最小填充率*/
    double MoonFrameMaxROF         = 0; /**< 半月框最大填充率*/
    double InsideMoonWidth         = 0; /**< 内半月框宽度（mm）*/
    double InsideMoonLength        = 0; /**< 内半月框长度（mm）*/
    double OutsideMoonWidth        = 0; /**< 外半月框宽度（mm）*/
    double OutsideMoonHeight       = 0; /**< 外半月框高度（mm）*/
    double FanbladeCenter2InsideMoontop  = 0; /**< 扇叶中心到内半月框上端的距离*/
    double FanbladeCenter2OutsideMoontop = 0; /**< 扇叶中心到外半月框下端的距离*/
    double FanBladeCenter_Outside_Inside_1 = 0; /**< 扇叶中心在外半月框和内半月框中心连线上的比例1*/
    double FanBladeCenter_Outside_Inside_2 = 0; /**< 扇叶中心在外半月框和内半月框中心连线上的比例2*/
  };

  struct TimeCost
  {
    std::chrono::milliseconds ImgCallback_cost;
  };

  struct CameraInfo
  {
    cv::Mat camera_matrix_ = cv::Mat::ones(3, 3, CV_64FC1);
    std::vector<double> dist_coeffs_;
    size_t image_width_  = 1440;
    size_t image_height_ = 1080;
  };

  struct DebugParams
  {
    bool raw_img_        = false; /**< 是否显示接收到的原图*/
    bool bin_img_        = false; /**< 是否显示二值化后的灰度图*/
    bool con_img_        = false; /**< 是否显示轮廓图*/
    bool rec_img_        = true;  /**< 是否显示识别图*/
    bool record_rec_img_ = false; /**< 录制识别图*/
    bool show_pnp        = true;  /**< 是否显示pnp解算目标坐标轴*/
    bool show_err_contours_ = true;  /**< 是否显示错误轮廓*/
  };

private:
  const double IOU_thre_ = 0.5;

  bool debug_    = true;  /**< 总调试开关，只有将其开启后调试启动参数才有效*/
  bool finded_waterfall_light_ = false; /**< 标记是否找到R，若未找到则无法进行下一步识别*/
  bool matched_fanblade_ = false; /**< 判断当前帧下是否查找到扇叶*/
  LightColor rune_color_;        /**< 能量机关颜色*/
  NodeStatus node_status_;

  int error_rotation_count_{};
  RuneRotationStatue rune_rotation_ = RuneRotationStatue::NONE;
  RuneRotationStatue last_rune_rotation_ = RuneRotationStatue::CCW;
  MoonFrame last_inside_moon_{};
  MoonFrame last_outside_moon_{};

  std::shared_ptr<FanBlade> fanblade_ = std::make_shared<FanBlade>(); /**< 记录当前帧查找到的扇叶*/
  std::shared_ptr<FanBlade> last_fanblade_ = std::make_shared<FanBlade>(); /**< 上一次识别到的扇叶*/

  rclcpp::TimerBase::SharedPtr time_{nullptr};
  rclcpp::TimerBase::SharedPtr status_timer_{nullptr};

  std::unique_ptr<TimeCost> timecost_                   = std::make_unique<TimeCost>();
  std::unique_ptr<CameraInfo> camera_info_              = std::make_unique<CameraInfo>();
  std::unique_ptr<RuneParams> rune_params_              = std::make_unique<RuneParams>();
  std::unique_ptr<DebugParams> debug_params_            = std::make_unique<DebugParams>();
  std::unique_ptr<RosTopicName> ros_topic_name_         = std::make_unique<RosTopicName>();
  std::unique_ptr<RosTopicChannels> ros_topic_channels_ = std::make_unique<RosTopicChannels>();
  std::shared_ptr<std::array<cv::Mat, 3>> image_channels_ =
    std::make_shared<std::array<cv::Mat, 3>>();

  using Hierarchies     = std::vector<cv::Vec4i>;
  using Contours        = std::vector<std::vector<cv::Point>>;
  using WaterfallLights = std::vector<WaterfallLight>;
  using MoonFrames      = std::vector<MoonFrame>;

  std::shared_ptr<Hierarchies> hierarchy_            = std::make_shared<Hierarchies>();
  std::shared_ptr<Contours> contours_                = std::make_shared<Contours>();
  std::shared_ptr<WaterfallLights> waterfall_lights_ = std::make_shared<WaterfallLights>();
  std::shared_ptr<MoonFrames> moonframes_            = std::make_shared<MoonFrames>();
  std::shared_ptr<cv::Mat> img_raw_                  = std::make_shared<cv::Mat>();
  std::shared_ptr<cv::Mat> img_bin_                  = std::make_shared<cv::Mat>();
  std::shared_ptr<cv::Mat> img_con_                  = std::make_shared<cv::Mat>();
  std::shared_ptr<cv::Mat> img_rec_                  = std::make_shared<cv::Mat>();
  std::shared_ptr<cv::VideoWriter> recorder_         = std::make_shared<cv::VideoWriter>();
  std::shared_ptr<SolvePose> solvepose_              = std::make_shared<SolvePose>();

  void ImgCallback(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg);

  void CameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg);

  void declareParams();

  void updateParams();

  void updateNodeStatus();

  void preprocess(const std::shared_ptr<cv::Mat> & rgb_img);

  bool findWaterfallLight();

  LightColor judgeColor(const std::array<cv::Mat, 3> image_channels, const cv::Rect2f & rect);

  bool judgeRuneColor(const WaterfallLights & waterfall_lights);

  void findMoonFrame();

  bool matchFanBlade();

  std::array<cv::Point2d, 4> findPnp2DPoints (const MoonFrame & inside_moon, const MoonFrame & outside_moon);

  static inline double
  getAngle(const cv::Point2f & center, const cv::Point2f & p1, const cv::Point2f & p2)
  {
    cv::Vec2d v1{p1.x - center.x, p1.y - center.y};
    cv::Vec2d v2{p2.x - center.x, p2.y - center.y};
    v1           = cv::normalize(v1);
    v2           = cv::normalize(v2);
    const double angle = std::acos(v1.dot(v2));
    if (!std::isnormal(angle))
    {
      throw std::runtime_error("getAngle Error: angle isnt normal!");
    }
    return angle;
  }

  /**
   * @brief 检查三点是否近似共线
   * 
   * @param tol 阈值
   * @return true 共线
   * @return false 不共线
   */
  static inline bool
  is_collinear(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3) 
  {
    cv::Vec2f v1{p2.x - p1.x, p2.y - p1.y};
    cv::Vec2f v2{p3.x - p2.x, p3.y - p2.y};
    v1 = cv::normalize(v1);
    v2 = cv::normalize(v2);
    const double angle = std::acos(v1.dot(v2));
    return std::abs(angle) < 2.0 / 180.0 * M_PI ||
           std::abs(angle) > 178.0 / 180.0 * M_PI;
  }

  /**
   * @brief 在图像上绘制识别错误的轮廓和相关参数
   *
   * @param img 输入的图像，轮廓将在此图像上绘制
   * @param Contour 要绘制的轮廓点集
   * @param TextLeftCenter 文本左上角中心点坐标，用于确定文本位置
   * @param param_name 错误参数名
   * @param value 相关参数值
   */
  static inline void
  drawWrongContours(
    bool                           is_draw,
    cv::Mat                      & img,
    const std::vector<cv::Point> & Contour,
    const cv::Point2f            & TextLeftCenter,
    const std::string            & param_name,
    const double                   value)
  {
    if (is_draw)
    {
      std::vector<std::vector<cv::Point>> wrongContours;
      wrongContours.push_back(Contour);
      cv::drawContours(img, wrongContours, -1, cv::Scalar(144, 238, 144), 1);
      cv::putText(
        img,
        param_name + ": " + std::to_string(value),
        cv::Point2f(TextLeftCenter.x - 40, TextLeftCenter.y - 5),
        cv::FONT_HERSHEY_SIMPLEX,
        0.6,
        cv::Scalar(255, 255, 255),
        1,
        4);
      wrongContours.pop_back();
    }
  }

  template <typename T>
  double square(const T & p) const
  {
    return p * p;
  }

  template <typename T>
  double dist2D(const T & p1, const T & p2) const
  {
    return std::sqrt(square(p1.x - p2.x) + square(p1.y - p2.y));
  }

  /**
   * @brief Used to compute the dot product of vector cross products
   *        val < 0 -> CW (p1 to p2)
   *        val > 0 -> CCW (p1 to p2)
   *        val = 0 -> collineation
   * 
   * @tparam T cv::Vec2f or cv::Vec2d
   * @param p1 vector_1
   * @param p2 vector_2
   * @return double: The dot product of the cross product
   */
  template <typename T>
  double crossProduct(const T & p1, const T &p2) const
  {
    return p1[0] * p2[1] - p1[1] * p2[0];
  }
};
}  // namespace rune

#endif  // RECOGNITION_NODE_HPP_
