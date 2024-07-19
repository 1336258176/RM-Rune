#ifndef PREDICTOR_HPP_
#define PREDICTOR_HPP_

#include <Eigen/Eigen>
#include <iostream>
#include <stdexcept>

#include <array>
#include <cmath>
#include <type_traits>

namespace rune
{
/**
 * @brief 用于描述扇叶状态
 */
enum class RuneStatue {
  NONE,  /**< 空*/
  SMALL, /**< 小符模式*/
  BIG    /**< 大符模式*/
};

/**
 * @brief 预测过程参考：https://github.com/HEU-Wings-of-Dream/Rune-lib，matlab上习惯采用行向量，而数学上习惯使用列向量，注意二者的转换。
 */
template <int queue_length = 200, int dw = 10>
class Predictor
{
private:
  double omega_small_ = static_cast<double>((M_PI) / 3.0); /**< 小符角速度*/
  double dw_          = static_cast<double>(dw);           /**< 每次迭代的角速度增量*/
  double omega_init_  = 1.942;                             /**< 角速度初始值(大符)*/
  double omega_       = omega_init_;                       /**< 角速度迭代值(大符)*/

  Eigen::Array<double, queue_length, 1> angles_array_; /**< 角度序列*/
  Eigen::Array<double, queue_length, 1> times_array_;  /**< 时间序列*/
  Eigen::Array<double, 4, 1> X_;
  Eigen::Matrix<double, queue_length, 1> Jac; /**< 雅可比矩阵*/

  void getOLS(double w);

  Eigen::Matrix<double, 1, queue_length> getFuncVal(double w);

  void getJacVal(double w);

  void calcGN();

public:
  void update(const std::array<double, queue_length> & angles,
              const std::array<double, queue_length> & times);

  double predict(const double & time,
                 const double & pre_time,
                 const RuneStatue & rune_statue,
                 const double & init_angle);

  double getOmega() const { return omega_; }
};

template <int queue_length, int dw>
void Predictor<queue_length, dw>::update(const std::array<double, queue_length> & angles,
                                         const std::array<double, queue_length> & times)
{
  angles_array_ = Eigen::Array<double, queue_length, 1>(angles.data());
  times_array_  = Eigen::Array<double, queue_length, 1>(times.data());
}

template <int queue_length, int dw>
void Predictor<queue_length, dw>::getOLS(double w)
{
  Eigen::MatrixXd a(queue_length, 4);
  a.col(0) = Eigen::sin(w * times_array_).matrix();
  a.col(1) = Eigen::cos(w * times_array_).matrix();
  a.col(2) = times_array_.matrix();
  a.col(3).setOnes();

  X_ = ((a.transpose() * a).ldlt().solve(a.transpose() * angles_array_.matrix())).array();
}

template <int queue_length, int dw>
Eigen::Matrix<double, 1, queue_length> Predictor<queue_length, dw>::getFuncVal(double w)
{
  /* 列转行 */
  Eigen::Array<double, 1, queue_length> times_array  = times_array_.matrix().transpose().array();
  Eigen::Array<double, 1, queue_length> angles_array = angles_array_.matrix().transpose().array();

  return (X_(0) * Eigen::sin(w * times_array) + X_(1) * Eigen::cos(w * times_array) +
          X_(2) * times_array + X_(3) - angles_array)
    .matrix();
}

template <int queue_length, int dw>
void Predictor<queue_length, dw>::getJacVal(double w)
{
  for (int i = 0; i < queue_length; i++)
  {
    Jac(i) = times_array_(i) * X_(0) * std::cos(times_array_(i) * w) -
      times_array_(i) * X_(1) * std::sin(times_array_(i) * w);
  }
}

template <int queue_length, int dw>
void Predictor<queue_length, dw>::calcGN()
{
  // double dw_ = dw;
  getOLS(omega_);
  while (dw_ > 1e-4)
  {
    Eigen::Matrix<double, 1, queue_length> f = getFuncVal(dw_);
    getJacVal(omega_);

    dw_ = (Jac.transpose() * f.transpose()).coeff(0) / (-(Jac.transpose() * Jac)).coeff(0);

    double fix_omega = omega_ + 3 * dw_;
    if (!std::isnormal(fix_omega))
    {
      std::clog << "fix-omega isnt normal, use omega_init_ instead.";
      fix_omega = omega_init_;
    }
    omega_ = fix_omega;
    getOLS(omega_);
  }
}

/**
 * @brief 预测
 * 
 * @tparam queue_length 
 * @tparam dw 
 * @param time 初始时间 ms
 * @param pre_time 向前预测时间 ms
 * @param rune_statue 能量机关旋转状态
 * @param init_angle 初始角度 rad
 * @return double 预测角度增量 rad
 */
template <int queue_length, int dw>
double Predictor<queue_length, dw>::predict(const double & time,
                                            const double & pre_time,
                                            const RuneStatue & rune_statue,
                                            const double & init_angle)
{
  if (rune_statue == RuneStatue::BIG)
  {
    calcGN();
    const auto & pre_angle = std::abs(X_[0] * std::sin(omega_ * (time + pre_time)) +
                                      X_[1] * std::cos(omega_ * (time + pre_time)) +
                                      X_[2] * (time + pre_time) + X_[3] - init_angle);
    return pre_angle;
  } else if (rune_statue == RuneStatue::SMALL)
  {
    return omega_small_ * pre_time * 1.0e-3;
  } else
  {
    throw std::runtime_error("Rune-Predictor Error: RuneStatue does not match.");
  }
}

}  // namespace rune

#endif  // PREDICTOR_HPP_
