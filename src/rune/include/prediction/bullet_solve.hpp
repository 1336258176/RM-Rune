#ifndef FIRE_CONTROL_PACKAGE__BULLRT_SOLVE_HPP_
#define FIRE_CONTROL_PACKAGE__BULLRT_SOLVE_HPP_

#include <cmath>

class BulletSolve
{
private:
  const double bullet_model_k = 0.026;
  const double height_gain = 0.1;
  const double gain = 0.6;
public:
  constexpr double rad2deg(double rad)
  {
    return rad / M_PI * 180.0;
  }
  double solveFlyTime(double x, double bullet_v, double pitch)
  {
    const double bullet_v_x = bullet_v * std::cos(pitch);
    double bullet_fly_time = (std::exp(bullet_model_k * x) - 1) / (bullet_model_k * bullet_v_x);
    if (1.0 < bullet_fly_time) bullet_fly_time = 1;
    if (!std::isnormal(bullet_fly_time)) bullet_fly_time = 0.0;

    return bullet_fly_time;
  }
  double bulletCompensation(double x, double y, float vy, double bullet_v, double& bullet_fly_time)
  {
    double out_x = x;
    double out_y = y;

    for (int i = 0; i < 20; ++i)
    {
      double pitch = std::atan2(out_y, out_x);
      bullet_fly_time = solveFlyTime(x, bullet_v, pitch);

      if (1e-4 > std::abs(bullet_fly_time))
      {
        bullet_fly_time = 0;
        return std::atan2(y, x);
      }

      // WHX old Model
      {
        double dy{};
        if (0.6 < y)
        {
          dy = y + vy*bullet_fly_time + height_gain*(y - 0.6) - (bullet_v*std::sin(pitch)*bullet_fly_time - 4.9*bullet_fly_time*bullet_fly_time);
        }
        else
        {
          dy = y + vy*bullet_fly_time - (bullet_v*std::sin(pitch)*bullet_fly_time - 4.9*bullet_fly_time*bullet_fly_time);
        }

        out_y += dy * gain;

        if (0.005 > std::abs(dy)) break;
      }
    }
    return std::atan2(out_y, out_x);
  }
  BulletSolve() = default;
  ~BulletSolve() = default;
};





#endif // FIRE_CONTROL_PACKAGE__BULLRT_SOLVE_HPP_