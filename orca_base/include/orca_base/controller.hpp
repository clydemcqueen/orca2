#ifndef ORCA_BASE_CONTROLLER_HPP
#define ORCA_BASE_CONTROLLER_HPP

#include "orca_base/base_context.hpp"
#include "orca_base/geometry.hpp"
#include "orca_base/pid.hpp"

namespace orca_base
{

  //=====================================================================================
  // Controller takes a planned pose and an estimated pose and uses 4 pid controllers to
  // compute an acceleration.
  //=====================================================================================

  class Controller
  {
  protected:

    // PID controllers
    pid::Controller x_controller_;
    pid::Controller y_controller_;
    pid::Controller z_controller_;
    pid::Controller yaw_controller_;

  public:

    explicit Controller(const BaseContext &cxt);

    // Calc u_bar
    // TODO use std::chrono::milliseconds instead of double
    virtual void calc(double dt, const Pose &plan, const Pose &estimate, const Acceleration &ff, Acceleration &u_bar);
  };

  //=====================================================================================
  // CalmController tries to reduce the number of times the thrusters start, stop and
  // flip directions.
  //=====================================================================================

  class CalmController : public Controller
  {
  public:

    explicit CalmController(const BaseContext &cxt, const Pose &plan) : Controller(cxt)
    {}

    void calc(double dt, const Pose &plan, const Pose &estimate, const Acceleration &ff, Acceleration &u_bar) override;
  };

} // namespace pid

#endif // ORCA_BASE_CONTROLLER_HPP