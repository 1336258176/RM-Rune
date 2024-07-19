#ifndef PREDICTOR_CERES_HPP_
#define PREDICTOR_CERES_HPP_

#include <ceres/ceres.h>

#include <Eigen/Eigen>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>

namespace rune
{
/**
 * @brief 用于描述扇叶状态
 */
enum class RuneStatue
{
  NONE,  /**< 空*/
  SMALL, /**< 小符模式*/
  BIG    /**< 大符模式*/
};

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

template <int queue_length = 200>
class PredictorCeres
{
public:
  // func  --> angle = B * sin(omega * t) + C * cos(omega * t) + b * t + c
  // params --> [B, omega, C, b, c]
  struct CostFunctor
  {
    CostFunctor(double angle, double time) : angle_(angle), time_(time) {}

    template <typename T>
    bool operator()(const T * const params, T * residual) const
    {
      residual[0] =
        angle_ - (params[0] * ceres::sin(params[1] * time_) +
                  params[2] * ceres::cos(params[1] * time_) + params[3] * time_ + params[4]);
      return true;
    }

  private:
    const double angle_;
    const double time_;
  };

  void update(
    const std::array<double, queue_length> & angles,
    const std::array<double, queue_length> & times);

  double predict(const double time,
                 const double pre_time,
                 const RuneStatue & rune_statue,
                 const double init_angle);

  double predictFixed(const double now_time, const double pre_time, const double init_angle);

  size_t getTimeCost() const { return time_cost_; }

  double getOmega() const { return params_[1]; }

  void getParams(double (&params)[5]) const
  {
    std::copy(std::begin(params_), std::end(params_), std::begin(params));
  }

private:
  void solve();

  bool isnormal() const;

  void reset()
  {
    params_[0] = 0.0;
    params_[1] = 1.942;
    params_[2] = 0.0;
    params_[3] = 0.0;
    params_[4] = 0.0;
    is_fixed = false;
  }

  double spd_small_ = static_cast<double>((M_PI) / 3.0); /**< 小符角速度*/

  size_t time_cost_{};
  bool is_fixed{false};

  double angles_sequence_[queue_length]{};
  double times_sequence_[queue_length]{};
  double params_[5]{0.0, 1.942, 0.0, 0.0, 0.0};

  ceres::Problem problem_;
  ceres::Solver::Options options_;
  ceres::Solver::Summary summary_;
};

template <int queue_length>
void PredictorCeres<queue_length>::update(
  const std::array<double, queue_length> & angles, const std::array<double, queue_length> & times)
{
  std::copy(angles.begin(), angles.end(), std::begin(angles_sequence_));
  std::copy(times.begin(), times.end(), std::begin(times_sequence_));
}

template <int queue_length>
void PredictorCeres<queue_length>::solve()
{
  Timer timer{};

  for (int i = 0; i < queue_length; ++i)
  {
    problem_.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CostFunctor, 1, 5>(
        new CostFunctor(angles_sequence_[i], times_sequence_[i])),
      nullptr, params_);
  }

  options_.linear_solver_type = ceres::DENSE_QR;
  options_.minimizer_progress_to_stdout = false;

  ceres::Solve(options_, &problem_, &summary_);

  time_cost_ = timer.end();
}

template <int queue_length>
bool PredictorCeres<queue_length>::isnormal() const
{
  if (
    !std::isnormal(params_[0]) || !std::isnormal(params_[1]) || !std::isnormal(params_[2]) ||
    !std::isnormal(params_[3]) || !std::isnormal(params_[4]))
  {
    return false;
  }

  double init_params[5]{0.0, 1.942, 0.0, 0.0, 0.0};
  if (std::equal(std::begin(params_), std::end(params_), std::begin(init_params)))
  {
    return false;
  }

  return true;
}

template <int queue_length>
double PredictorCeres<queue_length>::predict(const double time,
                                             const double pre_time,
                                             const RuneStatue & rune_statue,
                                             const double init_angle)
{
  if (rune_statue == RuneStatue::BIG)
  {
    try
    {
      solve();
    }
    catch (const std::exception & e)
    {
      throw std::runtime_error("Rune-Predictor-Ceres Error: solve failed.");
    }

    double all_time = time + pre_time + time_cost_;

    double pre_angle = std::abs(
      params_[0] * ceres::sin(params_[1] * all_time) +
      params_[2] * ceres::cos(params_[1] * all_time) + params_[3] * all_time + params_[4] -
      init_angle);

    if (!std::isnormal(pre_angle))
    {
      pre_angle = init_angle;
      throw std::runtime_error("Rune-Predictor-Ceres Error: pre_angle is not normal.");
    }

    return pre_angle;
  }
  else if (rune_statue == RuneStatue::SMALL)
  {
    return spd_small_ * pre_time * 1.0e-3;
  }
  else
  {
    throw std::runtime_error("Rune-Predictor-Ceres Error: RuneStatue does not match.");
  }
}

template <int queue_length>
double PredictorCeres<queue_length>::predictFixed(const double now_time,
                                                  const double pre_time,
                                                  const double init_angle)
{
  if (!is_fixed)
  {
    reset();
    solve();

    is_fixed = true;
    if (!isnormal())
    {
      is_fixed = false;
      return init_angle;
    }
  }

  double all_time = now_time + pre_time + time_cost_;
  double pre_angle = params_[0] * ceres::sin(params_[1] * all_time) +
                     params_[2] * ceres::cos(params_[1] * all_time) + params_[3] * all_time +
                     params_[4];
  if (!std::isnormal(pre_angle))
  {
    pre_angle = init_angle;
    throw std::runtime_error("Rune-Predictor-Ceres Error: pre_angle is not normal.");
  }
  return pre_angle;
}

}  // namespace rune

#endif  // PREDICTOR_CERES_HPP_