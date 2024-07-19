#ifndef PREDICTOR_GD_HPP_
#define PREDICTOR_GD_HPP_

#include <Eigen/Eigen>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace rune
{
class Timer
{
public:
  Timer() { start_ = std::chrono::high_resolution_clock::now(); }
  ~Timer() = default;

  size_t end()
  {
    end_ = std::chrono::high_resolution_clock::now();
    return static_cast<size_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(end_ - start_).count());
  }

private:
  std::chrono::high_resolution_clock::time_point start_;
  std::chrono::high_resolution_clock::time_point end_;
};

class PredictorGD
{
public:
  PredictorGD()  = default;
  ~PredictorGD() = default;

  void solve();

  void loadData(const std::array<double, 200> & angles, const std::array<double, 200> & times);

  size_t getCostTime() const { return time_cost_; }

  void getParams(double (&params)[5]) const
  {
    for (int i = 0; i < 5; i++)
    {
      params[i] = params_(i);
    }
  }

private:
  size_t time_cost_           = 0;
  const double learning_rate_ = 0.1;
  const int min_iter_         = 100;

  std::array<double, 200> angles_{};
  std::array<double, 200> times_{};

  // params --> B C omega b c
  Eigen::Matrix<double, 5, 1> params_{1, 1, 1.942, 1, 0};

  Eigen::Matrix<double, 5, 1> val_{0.0, 0.0, 0.0, 0.0, 0.0};

  void getVal(const double y, const double t);

  void updata();
};

void PredictorGD::loadData(const std::array<double, 200> & angles,
                           const std::array<double, 200> & times)
{
  std::copy(angles.begin(), angles.end(), angles_.begin());
  std::copy(times.begin(), times.end(), times_.begin());
}

void PredictorGD::getVal(const double y, const double t)
{
  // B
  val_(0) = -2.0 * std::sin(params_(2) * t) *
    (y - params_(0) * std::sin(params_(2) * t) - params_(1) * std::cos(params_(2) * t) -
     params_(3) * t - params_(4));
  if (!std::isnormal(val_(0)))
  {
    throw std::runtime_error("val_(0) is not normal");
  }

  // C
  val_.coeffRef(1) = -2.0 * std::cos(params_(2) * t) *
    (y - params_(0) * std::sin(params_(2) * t) - params_(1) * std::cos(params_(2) * t) -
     params_(3) * t - params_(4));
  if (!std::isnormal(val_(1)))
  {
    throw std::runtime_error("val_(1) is not normal");
  }

  // omega
  val_.coeffRef(2) = 2.0 *
    (y - params_(0) * std::sin(params_(2) * t) - params_(1) * std::cos(params_(2) * t) -
     params_(3) * t - params_(4)) *
    (-params_(0) * t * std::cos(params_(2) * t) + params_(1) * t * std::sin(params_(2) * t));
  if (!std::isnormal(val_(2)))
  {
    throw std::runtime_error("val_(2) is not normal");
  }

  // b
  val_.coeffRef(3) = -2.0 * t *
    (y - params_(0) * std::sin(params_(2) * t) - params_(1) * std::cos(params_(2) * t) -
     params_(3) * t - params_(4));
  if (!std::isnormal(val_(3)))
  {
    throw std::runtime_error("val_(3) is not normal");
  }

  // c
  val_.coeffRef(4) = -2.0 *
    (y - params_(0) * std::sin(params_(2) * t) - params_(1) * std::cos(params_(2) * t) -
     params_(3) * t - params_(4));
  if (!std::isnormal(val_(4)))
  {
    throw std::runtime_error("val_(4) is not normal");
  }
}

void PredictorGD::updata() { params_ -= learning_rate_ * val_; }

void PredictorGD::solve()
{
  Timer time{};

  int iter = 0;
  do
  {
    getVal(angles_[iter], times_[iter]);

    updata();

    iter++;
  } while (iter < min_iter_);

  time_cost_ = time.end();
}
}  // namespace rune

#endif  // !PREDICTOR_GD_HPP_
