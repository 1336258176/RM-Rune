#include <ament_index_cpp/get_package_share_directory.hpp>

#include <opencv2/opencv.hpp>

#include <array>
#include <string>

namespace rune
{
class SolvePose
{
private:
  cv::Mat camera_matrix_ = cv::Mat::ones(3, 3, CV_64FC1);
  std::vector<double> dist_coeffs_{};

public:
  SolvePose()  = default;
  ~SolvePose() = default;

  SolvePose(const std::string config_filename);
  SolvePose(const cv::Mat & camera_matrix, const std::vector<double> & dist_coeffs);
  SolvePose(const std::array<double, 9> & camera_matrix, const std::vector<double> & dist_coeffs);

  void update(const cv::Mat & camera_matrix, const std::vector<double> & dist_coeffs);

  void solvePnP(const std::array<cv::Point3f, 4> & object_points,
                const std::array<cv::Point2f, 4> & image_points,
                cv::Mat & rotation_vector,
                cv::Mat & translation_vector);
};
}  // namespace rune