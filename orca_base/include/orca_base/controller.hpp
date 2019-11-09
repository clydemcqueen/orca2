#ifndef ORCA_BASE_CONTROLLER_HPP
#define ORCA_BASE_CONTROLLER_HPP

#include "orca_base/base_context.hpp"
#include "orca_base/geometry.hpp"
#include "orca_base/pid.hpp"

namespace orca_base
{

  //=====================================================================================
  // ControllerBase defines and initializes 4 PID controllers.
  //=====================================================================================

  class ControllerBase
  {
  protected:

    // PID controllers
    pid::Controller x_controller_;
    pid::Controller y_controller_;
    pid::Controller z_controller_;
    pid::Controller yaw_controller_;

  public:

    explicit ControllerBase(const BaseContext &cxt) :
      x_controller_{false, cxt.auv_x_pid_ku_, cxt.auv_x_pid_tu_},
      y_controller_{false, cxt.auv_y_pid_ku_, cxt.auv_y_pid_tu_},
      z_controller_{false, cxt.auv_z_pid_ku_, cxt.auv_z_pid_tu_},
      yaw_controller_{true, cxt.auv_yaw_pid_ku_, cxt.auv_yaw_pid_tu_}
    {}

    // Calc u_bar
    virtual void calc(const BaseContext &cxt, double dt, const Pose &plan, const nav_msgs::msg::Odometry &estimate,
                      const Acceleration &ff, Acceleration &u_bar) = 0;
  };

  //=====================================================================================
  // SimpleController uses 4 pid controllers to compute u_bar.
  //=====================================================================================

  class SimpleController : public ControllerBase
  {
  public:

    explicit SimpleController(const BaseContext &cxt) : ControllerBase{cxt}
    {}

    // Calc u_bar
    void calc(const BaseContext &cxt, double dt, const Pose &plan, const nav_msgs::msg::Odometry &estimate,
              const Acceleration &ff, Acceleration &u_bar) override;
  };

  //=====================================================================================
  // IgnoreEstimateController ignores the estimate, so u_bar = ff.
  //=====================================================================================

  class IgnoreEstimateController : public ControllerBase
  {
  public:

    explicit IgnoreEstimateController(const BaseContext &cxt) : ControllerBase{cxt}
    {}

    // Calc u_bar
    void calc(const BaseContext &cxt, double dt, const Pose &plan, const nav_msgs::msg::Odometry &estimate,
              const Acceleration &ff, Acceleration &u_bar) override
    { u_bar = ff; }
  };

  //=====================================================================================
  // DeadzoneController turns off the PID controllers if the error is small
  //=====================================================================================

  class DeadzoneController : public ControllerBase
  {
  public:

    explicit DeadzoneController(const BaseContext &cxt) : ControllerBase{cxt}
    {}

    void calc(const BaseContext &cxt, double dt, const Pose &plan, const nav_msgs::msg::Odometry &estimate,
              const Acceleration &ff, Acceleration &u_bar) override;
  };

  //=====================================================================================
  // JerkController limits jerk
  //=====================================================================================

  class JerkController : public ControllerBase
  {
  private:
    // Keep previous u_bar
    Acceleration prev_u_bar_;

  public:

    explicit JerkController(const BaseContext &cxt) : ControllerBase{cxt}
    {}

    void calc(const BaseContext &cxt, double dt, const Pose &plan, const nav_msgs::msg::Odometry &estimate,
              const Acceleration &ff, Acceleration &u_bar) override;
  };

  //=====================================================================================
  // BestController uses a deadzone and limits jerk
  //=====================================================================================

  class BestController : public ControllerBase
  {
  private:
    // Keep previous u_bar
    Acceleration prev_u_bar_;

  public:

    explicit BestController(const BaseContext &cxt) : ControllerBase{cxt}
    {}

    void calc(const BaseContext &cxt, double dt, const Pose &plan, const nav_msgs::msg::Odometry &estimate,
              const Acceleration &ff, Acceleration &u_bar) override;
  };

  //=====================================================================================
  // DepthController controls depth, and ignores error on x, y and yaw
  //=====================================================================================

  class DepthController : public ControllerBase
  {
  public:

    explicit DepthController(const BaseContext &cxt) : ControllerBase{cxt}
    {}

    void calc(const BaseContext &cxt, double dt, const Pose &plan, const nav_msgs::msg::Odometry &estimate,
              const Acceleration &ff, Acceleration &u_bar) override;
  };

} // namespace pid

#endif // ORCA_BASE_CONTROLLER_HPP