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

    // Standard constructor
    Controller(bool angle, double Kp, double Ki, double Kd)
    {
      angle_ = angle;
      Kp_ = Kp;
      Ki_ = Ki;
      Kd_ = Kd;
    }

    // Ziegler–Nichols constructor
    // https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
    Controller(bool angle, double Ku, double Tu)
    {
      angle_ = angle;

#ifdef CLASSIC
      // Classic
      Kp_ = 0.6 * Ku;
      Ki_ = 1.2 * Ku / Tu;
      Kd_ = 3 * Ku * Tu / 40;
#else
      // P controller
      // This isn't perfect, but it's stable
      Kp_ = 0.5 * Ku;
      Ki_ = 0;
      Kd_ = 0;
#endif
    }

    // Set target
    void set_target(double target)
    {
      if (angle_) {
        norm_angle(target);
      }

      if (std::abs(target - target_) > 0.001) { // TODO constant
        // std::cout << "set target, from " << target_ << " to " << target << std::endl;
        target_ = target;
        prev_error_ = 0;
        integral_ = 0;
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

    double target()
    { return target_; }
  };

} // namespace pid

#endif // ORCA_BASE_PID_HPP