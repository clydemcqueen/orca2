#ifndef ORCA_BASE_CONTROLLER_HPP
#define ORCA_BASE_CONTROLLER_HPP

#include "orca_shared/geometry.hpp"

#include "orca_base/base_context.hpp"
#include "orca_base/pid.hpp"

namespace orca_base
{

  //=====================================================================================
  // PoseController uses 4 pid controllers in the world frame to compute efforts
  //=====================================================================================

  class PoseController
  {
    const BaseContext &cxt_;

    // PID controllers
    pid::Controller x_controller_;
    pid::Controller y_controller_;
    pid::Controller z_controller_;
    pid::Controller yaw_controller_;

  public:

    explicit PoseController(const BaseContext &cxt);

    void calc(const rclcpp::Duration &d, const orca::FP &plan, const orca::FP &estimate, const orca::Acceleration &ff,
              orca::Pose &error, orca::Efforts &efforts);
  };

#if 0
  //=====================================================================================
  // IgnoreEstimateController ignores the estimate, so u_bar = ff.
  //=====================================================================================

  class IgnoreEstimateController : public ControllerBase
  {
  public:

    explicit IgnoreEstimateController(const BaseContext &cxt) : ControllerBase{cxt}
    {}

    bool dead_reckoning() override
    { return true; }

    void calc(const BaseContext &cxt, double dt, const orca::FP &plan, const orca::FP &estimate,
              const orca::Acceleration &ff, orca::Acceleration &u_bar) override
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

    bool dead_reckoning() override
    { return false; }

    void calc(const BaseContext &cxt, double dt, const orca::FP &plan, const orca::FP &estimate,
              const orca::Acceleration &ff, orca::Acceleration &u_bar) override;
  };

  //=====================================================================================
  // JerkController limits jerk
  //=====================================================================================

  class JerkController : public ControllerBase
  {
  private:
    // Keep previous u_bar
    orca::Acceleration prev_u_bar_;

  public:

    explicit JerkController(const BaseContext &cxt) : ControllerBase{cxt}
    {}

    bool dead_reckoning() override
    { return false; }

    void calc(const BaseContext &cxt, double dt, const orca::FP &plan, const orca::FP &estimate,
              const orca::Acceleration &ff, orca::Acceleration &u_bar) override;
  };

  //=====================================================================================
  // BestController uses a deadzone and limits jerk
  //=====================================================================================

  class BestController : public ControllerBase
  {
  private:
    // Keep previous u_bar
    orca::Acceleration prev_u_bar_;

  public:

    explicit BestController(const BaseContext &cxt) : ControllerBase{cxt}
    {}

    bool dead_reckoning() override
    { return false; }

    void calc(const BaseContext &cxt, double dt, const orca::FP &plan, const orca::FP &estimate,
              const orca::Acceleration &ff, orca::Acceleration &u_bar) override;
  };

  //=====================================================================================
  // DepthController controls depth, and ignores error on x, y and yaw
  //=====================================================================================

  class DepthController : public ControllerBase
  {
  public:

    explicit DepthController(const BaseContext &cxt) : ControllerBase{cxt}
    {}

    bool dead_reckoning() override
    { return true; }

    void calc(const BaseContext &cxt, double dt, const orca::FP &plan, const orca::FP &estimate,
              const orca::Acceleration &ff, orca::Acceleration &u_bar) override;
  };
#endif

  //=====================================================================================
  // Observation controller uses fiducial_vlam observations, not poses
  // Observations are in the body frame, not the world frame
  // The distance calculation is quite noisy, so don't use a forward_controller_
  // Re-visit this if obs.destination is filtered
  //=====================================================================================

  class ObservationController
  {
    const BaseContext &cxt_;

    // PID controllers
    pid::Controller vertical_controller_;
    pid::Controller yaw_controller_;

  public:

    explicit ObservationController(const BaseContext &cxt);

    // The observation is pretty noisy for z, so pass in z data from the barometer
    void
    calc(const rclcpp::Duration &d, const orca::Observation &plan, double plan_z, const orca::Observation &estimate,
         double estimate_z, const orca::AccelerationBody &ff, orca::Pose &error, orca::Efforts &efforts);
  };

} // namespace orca_base

#endif // ORCA_BASE_CONTROLLER_HPP