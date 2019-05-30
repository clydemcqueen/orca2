#ifndef ORCA_BASE_PID_HPP
#define ORCA_BASE_PID_HPP

#include <math.h>

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

    // Standard constructor
    Controller(bool angle, double Kp, double Ki, double Kd)
    {
      angle_ = angle;
      Kp_ = Kp;
      Ki_ = Ki;
      Kd_ = Kd;
    }

    // Intuitive constructor
    Controller(bool angle, double damping_ratio, double natural_frequency)
    {
      angle_ = angle;
      Kp_ = natural_frequency * natural_frequency * (1 + 2 * damping_ratio);
      Ki_ = natural_frequency * natural_frequency * natural_frequency;
      Kd_ = natural_frequency * (1 + 2 * damping_ratio);
    }

    // Set target
    void set_target(double target)
    {
      if (angle_) {
        norm_angle(target);
      }

      target_ = target;
      prev_error_ = 0;
      integral_ = 0;
    }

    // Run one calculation
    double calc(double state, double dt, double bias)
    {
      double error = target_ - state;

      if (angle_) {
        norm_angle(error);
      }

      integral_ = integral_ + (error * dt);
      double derivative = (error - prev_error_) / dt;
      prev_error_ = error;

      return Kp_ * error + Ki_ * integral_ + Kd_ * derivative + bias;
    }

    double target()
    { return target_; }
  };

} // namespace pid

#endif // ORCA_BASE_PID_HPP