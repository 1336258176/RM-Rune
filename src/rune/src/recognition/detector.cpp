/**
 * @file fan_blade.cpp
 * @author Bin Li (lybin1336258176@outlook.com)
 * @brief
 * @date 2023-10-10
 * @version 0.1
 *
 * @copyright Copyright (c) 2023 by Bin Li
 *
 */

// clang-format off
#include <cmath>
#include <stdexcept>
#include "recognition/recognition_node.hpp"
// clang-format on

namespace rune
{

void RecognitionNode::preprocess(const std::shared_ptr<cv::Mat> & img)
{
  if (img->empty())
  {
    throw std::runtime_error("The raw-img is empty !");
  }
  if (img->type() != CV_8UC3)
  {
    throw std::runtime_error("The raw-img is not CV_8UC3 !");
  }

  cv::split(*img, *image_channels_);
  cv::cvtColor(*img, *img_bin_, cv::COLOR_RGB2GRAY);

  if (img_bin_->type() != CV_8UC1)
  {
    throw std::runtime_error("The bin-img is not CV_8UC1 !");
  }

  cv::threshold(*img_bin_,
                *img_bin_,
                (rune_color_ == LightColor::RED ? rune_params_->BinartThres_Red
                                                : rune_params_->BinaryThres_Blue),
                255,
                cv::THRESH_BINARY);

  const cv::Mat structuring_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(*img_bin_, *img_bin_, cv::MORPH_OPEN, structuring_element, cv::Point(-1, -1), 1);
  cv::morphologyEx(*img_bin_, *img_bin_, cv::MORPH_CLOSE, structuring_element, cv::Point(-1, -1), 1);

  if (!contours_->empty())
  {
    contours_->clear();
  }
  if (!hierarchy_->empty())
  {
    hierarchy_->clear();
  }

  cv::findContours(*img_bin_, *contours_, *hierarchy_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  if (debug_ && debug_params_->con_img_)
  {
    cv::drawContours(*img_con_, *contours_, -1, cv::Scalar(201, 207, 54), 2);
  }
}

bool RecognitionNode::findWaterfallLight()
{
  if (contours_->empty() || hierarchy_->empty())
  {
    return false;
  }
  if (!waterfall_lights_->empty())
  {
    waterfall_lights_->clear();
  }

  for (size_t i = 0; i < contours_->size(); i++)
  {
    /* 寻找没有父轮廓和子轮廓的轮廓 */
    if ((*hierarchy_)[i][3] != -1 || (*hierarchy_)[i][2] != -1)
    {
      continue;
    }

    /* 面积排除噪声 */
    const double area = cv::contourArea(contours_->at(i));
    if (area < rune_params_->WaterfallLightMinArea ||
        area > rune_params_->WaterfallLightMaxArea)
    {
      // drawWrongContours(debug_params_->show_err_contours_,
      //                   *img_rec_,
      //                   contours_->at(i),
      //                   contours_->at(i).at(0),
      //                   "area",
      //                   area);
      continue;
    }

    /* 外接矩形宽高比排除噪声 */
    const cv::Rect boundingRect = cv::boundingRect(contours_->at(i));
    const double aspect_ratio   = static_cast<double>(std::min(boundingRect.width, boundingRect.height)) /
                                  static_cast<double>(std::max(boundingRect.width, boundingRect.height));
    if (aspect_ratio < rune_params_->WaterfallLightMinAspectRatio ||
        aspect_ratio > rune_params_->WaterfallLightMaxAspectRatio ||
        !std::isnormal(aspect_ratio))
    {
      drawWrongContours(debug_params_->show_err_contours_,
                        *img_rec_,
                        contours_->at(i),
                        contours_->at(i).at(0),
                        "asp",
                        aspect_ratio);
      continue;
    }

    /* 填充率排除噪声 */
    std::vector<cv::Point> hull;
    cv::convexHull(contours_->at(i), hull);
    const double hull_area = cv::contourArea(hull);
    const double rof = area / hull_area;
    if (rof < rune_params_->WaterfallLightMinROF ||
        rof > rune_params_->WaterfallLightMaxROF ||
        !std::isnormal(rof))
    {
      drawWrongContours(debug_params_->show_err_contours_,
                        *img_rec_,
                        contours_->at(i),
                        contours_->at(i).at(0),
                        "rof",
                        rof);
      continue;
    }

    /* 寻找R型圆框 */
    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(contours_->at(i), center, radius);

    waterfall_lights_->emplace_back(radius, aspect_ratio, boundingRect, center);
  }

  // 流水灯筛选
  if (waterfall_lights_->size() > 3)
  {
    // 等距性检验
    std::unordered_set<int> distance_points{};
    const int n_light = waterfall_lights_->size();
    for (int i = 0; i < n_light; i++)
    {
      for (int j = i + 1; j < n_light; j++)
      {
        for (int k = j + 1; k < n_light; k++)
        {
          const double dist_i_j = dist2D<cv::Point2f>(waterfall_lights_->at(i).center_,
                                                     waterfall_lights_->at(j).center_);
          const double dist_i_k = dist2D<cv::Point2f>(waterfall_lights_->at(i).center_,
                                                     waterfall_lights_->at(k).center_);
          const double dist_j_k = dist2D<cv::Point2f>(waterfall_lights_->at(j).center_,
                                                     waterfall_lights_->at(k).center_);
          std::vector<double> dist{dist_i_j, dist_i_k, dist_j_k};
          std::erase(dist, std::ranges::max(dist));
          if (std::abs(dist[0] - dist[1]) <= rune_params_->WaterfallLightMaxDistanceDiff)
          {
            distance_points.insert(i);
            distance_points.insert(j);
            distance_points.insert(k);
          }
        }
      }
    }

    // 共线性检验
    std::unordered_set<int> collinear_points{};
    std::vector<int> distance_points_idx(distance_points.begin(), distance_points.end());
    const int n_dist_points = distance_points_idx.size();
    for (int i = 0; i < n_dist_points; i++)
    {
      for (int j = i + 1; j < n_dist_points; j++)
      {
        for (int k = j + 1; k < n_dist_points; k++)
        {
          const auto & p1 = waterfall_lights_->at(distance_points_idx.at(i)).center_;
          const auto & p2 = waterfall_lights_->at(distance_points_idx.at(j)).center_;
          const auto & p3 = waterfall_lights_->at(distance_points_idx.at(k)).center_;
          if (is_collinear(p1, p2, p3))
          {
            collinear_points.insert(i);
            collinear_points.insert(j);
            collinear_points.insert(k);
          }
        }
      }
    }

    // 构造最大共线等距集
    WaterfallLights tmp_storage{};
    std::for_each(collinear_points.begin(),
                  collinear_points.end(),
                  [&tmp_storage, this](const int index) {
                    tmp_storage.emplace_back(waterfall_lights_->at(index));
                  });
    waterfall_lights_->clear();
    waterfall_lights_->shrink_to_fit();
    std::copy(tmp_storage.begin(), tmp_storage.end(), std::back_inserter(*waterfall_lights_));
  }

  if (debug_ && debug_params_->rec_img_)
  {
    for (const auto & r : (*waterfall_lights_))
    {
      cv::circle(*img_rec_, r.center_, r.radius_, cv::Scalar(0, 97, 255), 4);
      cv::circle(*img_rec_, r.center_, 3, cv::Scalar(0, 255, 0), -1);
      cv::putText(*img_rec_,
                  "(" + std::to_string(static_cast<int>(r.center_.x)) + "," +
                    std::to_string(static_cast<int>(r.center_.y)) + ")",
                  r.center_ + cv::Point2f(-20, -20),
                  cv::FONT_HERSHEY_TRIPLEX,
                  0.5,
                  cv::Scalar(255, 0, 255));
    }
  }

  if (waterfall_lights_->empty())
  {
    return false;
  }
  else
  {
    return true;
  }
}

void RecognitionNode::findMoonFrame()
{
  if (!moonframes_->empty())
  {
    moonframes_->clear();
  }

  for (size_t i = 0; i < contours_->size(); i++)
  {
    /* 寻找没有父轮廓和子轮廓的轮廓 */
    if ((*hierarchy_)[i][3] != -1 || (*hierarchy_)[i][2] != -1)
    {
      continue;
    }

    /* 面积排除噪声 */
    const double area = cv::contourArea(contours_->at(i));
    if (area < rune_params_->MoonFrameMinArea ||
        area > rune_params_->MoonFrameMaxArea)
    {
      // drawWrongContours(debug_params_->show_err_contours_,
      //                   *img_rec_,
      //                   contours_->at(i),
      //                   contours_->at(i).at(0),
      //                   "area",
      //                   area);
      continue;
    }

    /* 外接矩形宽高比排除噪声 */
    const cv::RotatedRect rotated_rect = cv::minAreaRect(contours_->at(i));
    const cv::Point2f center = rotated_rect.center;
    const double aspect_ratio = static_cast<double>(std::min(rotated_rect.size.width,
                                                             rotated_rect.size.height)) /
                                static_cast<double>(std::max(rotated_rect.size.width,
                                                             rotated_rect.size.height));
    if (aspect_ratio < rune_params_->MoonFrameMinAspectRatio ||
        aspect_ratio > rune_params_->MoonFrameMaxAspectRatio ||
        !std::isnormal(aspect_ratio))
    {
      drawWrongContours(debug_params_->show_err_contours_,
                        *img_rec_,
                        contours_->at(i),
                        contours_->at(i).at(0),
                        "asp",
                        aspect_ratio);
      continue;
    }

    /* 填充率排除噪声 */
    std::vector<cv::Point> hull;
    cv::convexHull(contours_->at(i), hull);
    const double hull_area = cv::contourArea(hull);
    const double t_tem_rof = area / hull_area;
    if (t_tem_rof < rune_params_->MoonFrameMinROF ||
        t_tem_rof > rune_params_->MoonFrameMaxROF ||
        !std::isnormal(t_tem_rof))
    {
      drawWrongContours(debug_params_->show_err_contours_,
                        *img_rec_,
                        contours_->at(i),
                        contours_->at(i).at(0),
                        "rof",
                        t_tem_rof);
      continue;
    }

    std::array<cv::Point2f, 4> points;
    rotated_rect.points(points.data());
    moonframes_->emplace_back(aspect_ratio, Statue::NO_MATCHING, center, points, rotated_rect);
  }

  if (debug_ && debug_params_->rec_img_)
  {
    for (const auto & t : (*moonframes_))
    {
      for (int i = 0; i < 4; i++)
      {
        cv::line(*img_rec_, t.points_[i], t.points_[(i + 1) % 4], cv::Scalar(26, 196, 82), 2);
        cv::circle(*img_rec_, t.points_[i], i + 1, cv::Scalar(0, 0, 255), 2);
      }
      cv::circle(*img_rec_, t.center_, 3, cv::Scalar(102, 245, 255), -1);
      // cv::putText(*img_rec_,
      //             "T",
      //             t.center_ + cv::Point2f(-20, -20),
      //             cv::FONT_HERSHEY_TRIPLEX,
      //             0.8,
      //             cv::Scalar(255, 0, 255));
    }
  }
}

bool RecognitionNode::matchFanBlade()
{
  if (!fanblade_.use_count())
  {
    fanblade_ = std::make_shared<FanBlade>();
  }

  if (!finded_waterfall_light_ || moonframes_->empty())
  {
    return false;
  }

  MoonFrame outside_moon{};
  MoonFrame inside_moon{};
  int waterfall_light_count = 0;
  bool is_matched = false;

  for (auto & t1 : *moonframes_)
  {
    for (auto & t2 : *moonframes_)
    {
      if (t1.center_ == t2.center_ || is_matched)
      {
        continue;
      }

      for (const auto & r : *waterfall_lights_)
      {
        const double angle = getAngle(r.center_, t1.center_, t2.center_);
        if (std::abs(angle) < 2.0 / 180.0 * M_PI)
        {
          waterfall_light_count++;
        }
        else
        {
          waterfall_light_count--;
        }

        const int num_waterfall_lights = waterfall_lights_->size();
        if (waterfall_light_count >= num_waterfall_lights / 2.0 &&
            waterfall_light_count >= rune_params_->MinWaterfallLightCount)
        {
          const WaterfallLight & waterfall_light = waterfall_lights_->at(num_waterfall_lights / 2);
          const double dis1 = dist2D<cv::Point2d>(waterfall_light.center_, t1.center_);
          const double dis2 = dist2D<cv::Point2d>(waterfall_light.center_, t2.center_);
          inside_moon = (dis1 < dis2) ? t1 : t2;
          outside_moon = (dis1 < dis2) ? t2 : t1;
          is_matched = true;
        }
      }

      if (!is_matched)
      {
        waterfall_light_count = 0;
      }
    }
  }

  if (!is_matched)
  {
    return false;
  }

  if (!judgeRuneColor(*waterfall_lights_))
  {
    RCLCPP_WARN(this->get_logger(), "dont match rune color!");
    return false;
  }

  const std::array<cv::Point2d, 4> pnp_2D_points = findPnp2DPoints(inside_moon, outside_moon);

  const cv::Point2f center(outside_moon.center_.x * rune_params_->FanBladeCenter_Outside_Inside_1 +
                           inside_moon.center_.x * rune_params_->FanBladeCenter_Outside_Inside_2,
                           outside_moon.center_.y * rune_params_->FanBladeCenter_Outside_Inside_1 +
                           inside_moon.center_.y * rune_params_->FanBladeCenter_Outside_Inside_2);

  /* build fanblade */
  fanblade_->empty_         = false;
  fanblade_->time_stamp_    = this->now(); /**< init*/
  fanblade_->inside_frame_  = inside_moon;
  fanblade_->outside_frame_ = outside_moon;
  fanblade_->center_        = center;               /**< assignment*/
  fanblade_->center_3D_     = cv::Point3f(0, 0, 0); /**< init*/
  fanblade_->pnp_detection_points_3D_[0] =
    cv::Point3f(rune_params_->OutsideMoonWidth / 2, -rune_params_->FanbladeCenter2OutsideMoontop, 0);
  fanblade_->pnp_detection_points_3D_[1] =
    cv::Point3f(-rune_params_->OutsideMoonWidth / 2, -rune_params_->FanbladeCenter2OutsideMoontop, 0);
  fanblade_->pnp_detection_points_3D_[2] =
    cv::Point3f(-rune_params_->InsideMoonWidth / 2, rune_params_->FanbladeCenter2OutsideMoontop, 0);
  fanblade_->pnp_detection_points_3D_[3] =
    cv::Point3f(rune_params_->InsideMoonWidth / 2, rune_params_->FanbladeCenter2OutsideMoontop, 0);
  std::copy(pnp_2D_points.begin(), pnp_2D_points.end(), fanblade_->pnp_detection_points_2D_.begin());

  if (!last_fanblade_->empty_)
  {
    const cv::Rect2f now_rect       = fanblade_->outside_frame_.rotated_rect_.boundingRect2f();
    const cv::Rect2f last_rect      = last_fanblade_->outside_frame_.rotated_rect_.boundingRect2f();
    const cv::Rect2f rect_and       = now_rect | last_rect; /**< 矩形并集*/
    const cv::Rect2f rect_intersect = now_rect & last_rect; /**< 矩形交集*/
    const double iou = static_cast<double>(rect_intersect.area()) / static_cast<double>(rect_and.area());
    if (iou > IOU_thre_)
    {
      fanblade_->fanblade_statue_ = FanbladeStatue::MATCHING;
      // RCLCPP_INFO(this->get_logger(), "now-fanblade matched last-fanblade, IOU: %lf", iou);
    } else
    {
      fanblade_->fanblade_statue_ = FanbladeStatue::NO_MATCHING;
      // RCLCPP_INFO(this->get_logger(), "now-fanblade dont match last-fanblade, IOU: %lf", iou);
    }
  } else
  {
    fanblade_->fanblade_statue_ = FanbladeStatue::NO_MATCHING;
  }

  if (debug_ && debug_params_->rec_img_)
  {
    // draw fanblade center
    cv::circle(*img_rec_, center, 5, cv::Scalar(150, 47, 235), -1);

    // draw matched moon and t
    for (int i = 0; i < 4; i++)
    {
      cv::line(*img_rec_, 
           inside_moon.points_[i], 
           inside_moon.points_[(i + 1) % 4],
           cv::Scalar(0, 97, 255),
           4);
      cv::circle(*img_rec_,
                 inside_moon.points_[i],
                 i + 1,
                 cv::Scalar(0, 0, 255),
                 4);
      cv::line(*img_rec_, 
           outside_moon.points_[i], 
           outside_moon.points_[(i + 1) % 4],
           cv::Scalar(0, 97, 255),
           4);
      cv::circle(*img_rec_,
                 outside_moon.points_[i],
                 i + 1,
                 cv::Scalar(0, 0, 255),
                 4);
    }
    cv::circle(*img_rec_,
               inside_moon.center_,
               3,
               cv::Scalar(102, 245, 255),
               -1);
    cv::putText(*img_rec_,
                "T",
                inside_moon.center_ + cv::Point2f(-20, -20),
                cv::FONT_HERSHEY_TRIPLEX,
                0.8,
                cv::Scalar(255, 0, 255));
    cv::circle(*img_rec_,
               outside_moon.center_,
               3,
               cv::Scalar(102, 245, 255),
               -1);
    cv::putText(*img_rec_,
                "moon",
                outside_moon.center_ + cv::Point2f(-20, -20),
                cv::FONT_HERSHEY_TRIPLEX,
                0.8,
                cv::Scalar(255, 0, 255));
  }

  return true;
}

RecognitionNode::LightColor RecognitionNode::judgeColor(const std::array<cv::Mat, 3> image_channels,
                                                        const cv::Rect2f & rect)
{
  const cv::Scalar scalar_red  = cv::mean(image_channels[2](rect));
  const cv::Scalar scalar_blue = cv::mean(image_channels[0](rect));

  return (scalar_red[0] > scalar_blue[0]) ? LightColor::RED : LightColor::BLUE;
}

std::array<cv::Point2d, 4> RecognitionNode::findPnp2DPoints (const MoonFrame & inside_moon, const MoonFrame & outside_moon)
{
  std::array<cv::Point2d, 4> pnp_2D_points;
  for(const auto & point : outside_moon.points_)
  {
    cv::Vec2d vec_Outside2Inside {inside_moon.center_.x - outside_moon.center_.x,
                                  inside_moon.center_.y - outside_moon.center_.y};
    cv::Vec2d vec_Moon2Point {point.x - outside_moon.center_.x,
                              point.y - outside_moon.center_.y};
    vec_Outside2Inside = cv::normalize(vec_Outside2Inside);
    vec_Moon2Point = cv::normalize(vec_Moon2Point);

    const double dot_val = vec_Outside2Inside.dot(vec_Moon2Point);
    if (dot_val < 0)
    {
      continue;
    }

    const double cross_product_val = crossProduct<cv::Vec2d>(vec_Outside2Inside, vec_Moon2Point);
    if (cross_product_val > 0)
    {
      pnp_2D_points.at(0) = point;
    }
    else if (cross_product_val < 0)
    {
      pnp_2D_points.at(1) = point;
    }
    else
    {
      throw std::runtime_error("find pnp 2D points error");
    }
  }
  for(const auto & point : inside_moon.points_)
  {
    cv::Vec2d vec_Inside2Outside {outside_moon.center_.x - inside_moon.center_.x,
                                  outside_moon.center_.y - inside_moon.center_.y};
    cv::Vec2d vec_Moon2Point {point.x - inside_moon.center_.x,
                              point.y - inside_moon.center_.y};
    vec_Inside2Outside = cv::normalize(vec_Inside2Outside);
    vec_Moon2Point = cv::normalize(vec_Moon2Point);

    const double dot_val = vec_Inside2Outside.dot(vec_Moon2Point);
    if (dot_val < 0)
    {
      continue;
    }

    const double cross_product_val = crossProduct<cv::Vec2d>(vec_Inside2Outside, vec_Moon2Point);
    if (cross_product_val > 0)
    {
      pnp_2D_points.at(2) = point;
    }
    else if (cross_product_val < 0)
    {
      pnp_2D_points.at(3) = point;
    }
    else
    {
      throw std::runtime_error("find pnp 2D points error");
    }
  }
  return pnp_2D_points;
}

bool RecognitionNode::judgeRuneColor(const WaterfallLights & waterfall_lights)
{
  int color_count = 0;
  for (const auto & waterfall_light : waterfall_lights)
  {
    bool is_my_rune = judgeColor(*image_channels_, waterfall_light.R_Rect_) == rune_color_;
    if (is_my_rune)
    {
      color_count++;
    }
    else
    {
      color_count--;
    } 
  }
  if (color_count < waterfall_lights.size() / 2.0 && !waterfall_lights.empty())
  {
    return false;
  }
  else
  {
    return true;
  }
}

}  // namespace rune
