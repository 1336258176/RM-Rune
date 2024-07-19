#ifndef KALMAN_FILTER_CV_MODEL_FORRUNE_HPP_
#define KALMAN_FILTER_CV_MODEL_FORRUNE_HPP_

#include <Eigen/Dense>
#include <chrono>
#include <cmath>

namespace cv_model
{

class EKF
{
public:
  //! State size
  static constexpr size_t N_State = 6;
  //! Measure size
  static constexpr size_t N_Measure = 3;
  //! Noise size
  static constexpr size_t N_Noise = 3;
  //! X-position
  static constexpr size_t X = 0;
  //! X-Velocity
  static constexpr size_t dX = 1;
  //! Y-position
  static constexpr size_t Y = 2;
  //! Y-Velocity
  static constexpr size_t dY = 3;
  //! Z-position
  static constexpr size_t Z = 4;
  //! Z-Velocity
  static constexpr size_t dZ = 5;

  //! State: x dx y dy z dz
  using State       = Eigen::Matrix<double, N_State, 1>;
  //! Measure: x y z
  using Measure     = Eigen::Matrix<double, N_Measure, 1>;
  using MatrixSS    = Eigen::Matrix<double, N_State, N_State>;
  using MatrixMS    = Eigen::Matrix<double, N_Measure, N_State>;
  using MatrixSM    = Eigen::Matrix<double, N_State, N_Measure>;
  using MatrixMM    = Eigen::Matrix<double, N_Measure, N_Measure>;
  using MatrixNoise = Eigen::Matrix<double, N_Noise, 1>;

  explicit EKF(const State & S0 = State::Zero())
  : S_e(S0), Q(MatrixSS::Identity()), P(MatrixSS::Identity() * 2345675433), R(MatrixMM::Identity())
  {
    ChiSquare_err_count_ = 0;
    ChiSquare_god_count_ = 0;
  }

  void predict(const std::chrono::microseconds & sampling_time)
  {
    const double t = sampling_time.count() / 1e6;  // seconds
    const double t2 = t * t;
    const double t3 = t * t * t;
    const double t4 = t * t * t * t;

    A << 1, t, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0,
         0, 0, 1, t, 0, 0,
         0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 1, t,
         0, 0, 0, 0, 0, 1;

    const double sx_2 = sigma_s_x;
    const double sy_2 = sigma_s_y;
    const double sz_2 = sigma_s_z;

    Q << t4 * sx_2 / 4, t3 * sx_2 / 2, 0,             0,             0,             0,
         t3 * sx_2 / 2, t2 * sx_2,     0,             0,             0,             0,
         0,             0,             t4 * sy_2 / 4, t3 * sy_2 / 2, 0,             0,
         0,             0,             t3 * sy_2 / 2, t2 * sy_2,     0,             0,
         0,             0,             0,             0,             t4 * sz_2 / 4, t3 * sz_2 / 2,
         0,             0,             0,             0,             t3 * sz_2 / 2, t2 * sz_2;

    // 时间更新
    S_p = A * S_e;
    P = A * P * A.transpose() + Q;
  }

  State update(
    const Measure & M,
    const Measure & Camera_M,
    const Eigen::Matrix<double, 3, 3> & rot_camera2odom = Eigen::Matrix<double, 3, 3>::Identity())
  {
    H << 1, 0, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0,
         0, 0, 0, 0, 1, 0;

    M_p << S_p(cv_model::EKF::X),
           S_p(cv_model::EKF::Y),
           S_p(cv_model::EKF::Z);

    const double c_x = Camera_M(0);
    const double c_y = Camera_M(1);
    const double c_z = Camera_M(2);
    const double camera_x_2 = c_x * c_x;
    const double tan_alpha = c_y / c_x;
    const double tan_beta = c_z / c_x;
    const double tan_alpha_2 = tan_alpha * tan_alpha;
    const double tan_beta_2 = tan_beta * tan_beta;
    const double cos_alpha = c_x / std::sqrt(c_y * c_y + c_x * c_x);
    const double cos_beta = c_x / std::sqrt(c_z * c_z + c_x * c_x);
    const double cos_alpha_4 = cos_alpha * cos_alpha * cos_alpha * cos_alpha;
    const double cos_beta_4 = cos_beta * cos_beta * cos_beta * cos_beta;
    const double sigma_m_x_2 = sigma_m_x;
    const double sigma_m_alpha_2 = sigma_m_alpha;
    const double sigma_m_beta_2 = sigma_m_beta;
    const double sigma_1 = sigma_m_x_2 * tan_beta * tan_alpha;
    const double sigma_2 = sigma_m_x_2 * tan_alpha;
    const double sigma_3 = sigma_m_x_2 * tan_beta;
    const double sigma_4 = sigma_m_x_2 * tan_alpha_2 + sigma_m_alpha_2 * camera_x_2 / cos_alpha_4;
    const double sigma_5 = sigma_m_x_2 * tan_beta_2 + sigma_m_beta_2 * camera_x_2 / cos_beta_4;

    Eigen::Matrix<double, 3, 3> R_c;
    R_c << sigma_m_x_2, sigma_2, sigma_3,
           sigma_2,     sigma_4, sigma_1,
           sigma_3,     sigma_1, sigma_5;

    // 状态更新
    R_ = rot_camera2odom * R_c * rot_camera2odom.transpose();
    K_ = P * H.transpose() * ((H * P * H.transpose() + R_).inverse());

    Measure z_residual = M - M_p;
    Chi_error=ChiSquareTest(z_residual);
    if (Chi_error > chi_square_threshold)
    {
      ++ChiSquare_err_count_;
      if(ChiSquare_err_count_ > 0)
      {
        P = MatrixSS::Identity() * 10e5;
        S_e = H.transpose() * M;
        ChiSquare_err_count_ = 0;
        ChiSquare_god_count_ = 0;
        return S_e;
      }
      else
      {
        S_e = S_p;
        return S_e;
      }
    }
    else
    {
      R=R_;
      K=K_;
      ChiSquare_god_count_++;
      if(ChiSquare_god_count_>15)
      {
        ChiSquare_god_count_ = 0;
        ChiSquare_err_count_ = 0;
      }
      S_e = S_p + K * (z_residual);
      P = (MatrixSS::Identity() - K * H) * P;
      return S_e;
    }
  }

  double ChiSquareTest(const Measure & errZ)
  {
    const auto D = H * P * H.transpose() + R_;
    return errZ.transpose() * D.inverse() * errZ;
  }

  bool isnormal()
  {
    if (!S_e.array().isFinite().any()) return false;
    if (!S_p.array().isFinite().any()) return false;
    if (!M_p.array().isFinite().any()) return false;
    if (!A.array().isFinite().any()) return false;
    if (!H.array().isFinite().any()) return false;
    if (!P.array().isFinite().any()) return false;
    if (!Q.array().isFinite().any()) return false;
    if (!R.array().isFinite().any()) return false;
    if (!K.array().isFinite().any()) return false;
    return true;
  }

  void updateParams(
    const double & s_x,
    const double & s_y,
    const double & s_z,
    const double & m_x,
    const double & m_alpha,
    const double & m_beta,
    const double & chi_square_thre)
  {
    sigma_s_x = s_x;
    sigma_s_y = s_y;
    sigma_s_z = s_z;
    sigma_m_x = m_x;
    sigma_m_alpha = m_alpha;
    sigma_m_beta = m_beta;
    chi_square_threshold = chi_square_thre;
  }

  /**
   * @brief Obtain posterior states
   * 
   * @return Type: class Eigen::Matrix<double, 6, 1>  State: x dx y dy z dz
   */
  State getS_e() const { return S_e; }

  /**
   * @brief Obtain prior states
   * 
   * @return Type: class Eigen::Matrix<double, 6, 1>  State: x dx y dy z dz 
   */
  State getS_p() const { return S_p; }  
  double get_Chi() const { return Chi_error; }

  /**
   * @brief Set the state variable covariance matrix
   * 
   * @param P0 Type: class Eigen::Matrix<double, 6, 6>
   */
  void setP(const MatrixSS & P0) { P = P0; }

private:
  State S_e;    //后验状态
  State S_p;    //先验状态
  Measure M_p;  //先验量测
  MatrixSS A;   //状态转移矩阵
  MatrixSS Q;   //过程噪声协方差
  MatrixSS P;   //状态变量协方差
  MatrixSM K_;   //卡尔曼增益
  MatrixSM K;   //卡尔曼增益
  MatrixMS H;   //量测矩阵
  MatrixMM R_;   //量测噪声协方差
  MatrixMM R;   //量测噪声协方差

  double ChiSquare_err_count_ = 0;
  double ChiSquare_god_count_ = 0;
  double Chi_error = 0;

  // params
  double sigma_s_x{};
  double sigma_s_y{};
  double sigma_s_z{};
  double sigma_m_x{};
  double sigma_m_alpha{};
  double sigma_m_beta{};
  double chi_square_threshold{};
};

}  // namespace cv_model
#endif  //KALMAN_FILTER_SPINNING_MODEL_HPP