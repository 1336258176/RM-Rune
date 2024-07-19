#include "recognition/solvepose.hpp"

namespace rune
{
SolvePose::SolvePose(const std::string config_filename)
{
  std::string filepath =
    ament_index_cpp::get_package_share_directory("rune") + "/config/" + config_filename;
  cv::FileStorage fs(filepath, cv::FileStorage::READ);

  if (!fs.isOpened())
  {
    throw std::runtime_error("Solvepose Error: filestorage failed to open TAML file.");
  }

  /* 如果直接以Mat类型读取，务必注意yaml文件格式要求 */
  cv::Mat dist_coeffs;
  fs["camera_matrix"] >> camera_matrix_;
  fs["distortion_coefficients"] >> dist_coeffs;

  for (int i = 0; i < dist_coeffs.cols; i++)
  {
    dist_coeffs_.emplace_back(dist_coeffs.at<double>(i));
  }

  fs.release();
}

SolvePose::SolvePose(const cv::Mat & camera_matrix, const std::vector<double> & dist_coeffs) :
camera_matrix_(camera_matrix), dist_coeffs_(dist_coeffs)
{
}

SolvePose::SolvePose(const std::array<double, 9> & camera_matrix,
                     const std::vector<double> & dist_coeffs) :
camera_matrix_(cv::Mat(camera_matrix)), dist_coeffs_(dist_coeffs)
{
}

void SolvePose::update(const cv::Mat & camera_matrix, const std::vector<double> & dist_coeffs)
{
  if (camera_matrix.empty()) return;
  if (dist_coeffs.empty()) return;

  camera_matrix_ = camera_matrix;
  dist_coeffs_   = dist_coeffs;
}

void SolvePose::solvePnP(const std::array<cv::Point3f, 4> & object_points,
                         const std::array<cv::Point2f, 4> & image_points,
                         cv::Mat & rotation_vector,
                         cv::Mat & translation_vector)
{
  // 检查输入
  if (object_points.empty() || image_points.empty())
  {
    throw std::runtime_error("object_points or image_points is empty.");
  }

  if (camera_matrix_.empty() || dist_coeffs_.empty())
  {
    throw std::runtime_error("camera_matrix or dist_coeffs is empty.");
  }

  // 解算目标位姿
  if (cv::solvePnP(object_points,
                   image_points,
                   camera_matrix_,
                   dist_coeffs_,
                   rotation_vector,
                   translation_vector,
                   false,
                   cv::SOLVEPNP_IPPE) == false)
  {
    if(rotation_vector.empty() || translation_vector.empty())
    {
      throw std::runtime_error("IPPE error: r_vec or t_vec is empty.");
    }else
    {
      throw std::runtime_error("IPPE error: other moudle error.");
    }
  }
}
}  // namespace rune