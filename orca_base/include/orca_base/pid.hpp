#ifndef ORCA_BASE_PID_HPP
#define ORCA_BASE_PID_HPP

#include <cmath>

namespace pid
{

  constexpr void norm_angle(double &a)
  {
    while (a < -M_PI) {
      a += 2 * M_PI;
    }
    while (a > M_PI) {
      a -= 2 * M_PI;
    }
  }

  class Controller
  {
  private:

    bool angle_; // True if we're controlling an angle [-pi, pi]
    double target_ = 0;
    double prev_error_ = 0;
    double integral_ = 0;
    double Kp_;
    double Ki_;
    double Kd_;

  public:

    Controller(bool angle, double Kp, double Ki, double Kd)
    {
      angle_ = angle;
      Kp_ = Kp;
      Ki_ = Ki;
      Kd_ = Kd;
    }

    void set_target(double target)
    {
      if (angle_) {
        norm_angle(target);
      }

      if (std::abs(target - target_) > 0.001) {
        // std::cout << "set target, from " << target_ << " to " << target << std::endl;
        target_ = target;
        // prev_error_ = 0;
        // integral_ = 0;
      }
    }

    // Run one calculation
    double calc(double state, double dt)
    {
      double error = target_ - state;

      if (angle_) {
        norm_angle(error);
      }

      integral_ = integral_ + (error * dt);
      double derivative = (error - prev_error_) / dt;
      prev_error_ = error;

      return Kp_ * error + Ki_ * integral_ + Kd_ * derivative;
    }

    double target() const
    { return target_; }
  };

} // namespace pid

#endif // ORCA_BASE_PID_HPP