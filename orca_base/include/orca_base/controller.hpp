#ifndef ORCA_BASE_CONTROLLER_HPP
#define ORCA_BASE_CONTROLLER_HPP

#include "orca_base/base_context.hpp"
#include "orca_base/geometry.hpp"
#include "orca_base/pid.hpp"

namespace orca_base
{

  //=====================================================================================
  // BaseController takes a planned pose and an estimated pose and uses 4 pid controllers to
  // compute an acceleration.
  //=====================================================================================

  class BaseController
  {
  protected:

    // PID controllers
    pid::Controller x_controller_;
    pid::Controller y_controller_;
    pid::Controller z_controller_;
    pid::Controller yaw_controller_;

  public:

    explicit BaseController(const BaseContext &cxt);

    // Calc u_bar
    // TODO use std::chrono::milliseconds instead of double
    virtual void calc(const BaseContext &cxt, double dt, const Pose &plan, const Pose &estimate, const Acceleration &ff,
                      Acceleration &u_bar);
  };

  //=====================================================================================
  // DeadzoneController turns off the PID controllers if the error is small
  //=====================================================================================

  class DeadzoneController : public BaseController
  {
  public:

    explicit DeadzoneController(const BaseContext &cxt) : BaseController{cxt}
    {}

    void calc(const BaseContext &cxt, double dt, const Pose &plan, const Pose &estimate, const Acceleration &ff,
              Acceleration &u_bar) override;
  };

  //=====================================================================================
  // JerkController limits jerk
  //=====================================================================================

  class JerkController : public BaseController
  {
  private:
    // Keep previous u_bar
    Acceleration prev_u_bar_;

  public:

    explicit JerkController(const BaseContext &cxt) : BaseController{cxt}
    {}

    void calc(const BaseContext &cxt, double dt, const Pose &plan, const Pose &estimate, const Acceleration &ff,
              Acceleration &u_bar) override;
  };

  //=====================================================================================
  // BestController uses a deadzone and limits jerk
  //=====================================================================================

  class BestController : public BaseController
  {
  private:
    // Keep previous u_bar
    Acceleration prev_u_bar_;

  public:

    explicit BestController(const BaseContext &cxt) : BaseController{cxt}
    {}

    void calc(const BaseContext &cxt, double dt, const Pose &plan, const Pose &estimate, const Acceleration &ff,
              Acceleration &u_bar) override;
  };

} // namespace pid

#endif // ORCA_BASE_CONTROLLER_HPP