#ifndef ORCA_BASE_CONTROLLER_HPP
#define ORCA_BASE_CONTROLLER_HPP

#include "orca_shared/geometry.hpp"

#include "orca_base/auv_context.hpp"
#include "orca_base/fp.hpp"
#include "orca_base/pid.hpp"

namespace orca_base
{

  //=====================================================================================
  // PoseController uses 4 pid controllers in the world frame to compute efforts
  //=====================================================================================

  class PoseController
  {
    const AUVContext &cxt_;

    // PID controllers
    pid::Controller x_controller_;
    pid::Controller y_controller_;
    pid::Controller z_controller_;
    pid::Controller yaw_controller_;

  public:

    explicit PoseController(const AUVContext &cxt);

    void calc(const rclcpp::Duration &d, const FP &plan, const FP &estimate, const orca::Acceleration &ff,
              orca::Efforts &efforts);
  };

#if 0
  //=====================================================================================
  // IgnoreEstimateController ignores the estimate, so u_bar = ff.
  //=====================================================================================

  class IgnoreEstimateController : public ControllerBase
  {
  public:

    explicit IgnoreEstimateController(const AUVContext &cxt) : ControllerBase{cxt}
    {}

    bool dead_reckoning() override
    { return true; }

    void calc(const AUVContext &cxt, double dt, const FP &plan, const FP &estimate,
              const orca::Acceleration &ff, orca::Acceleration &u_bar) override
    { u_bar = ff; }
  };

  //=====================================================================================
  // DeadzoneController turns off the PID controllers if the error is small
  //=====================================================================================

  class DeadzoneController : public ControllerBase
  {
  public:

    explicit DeadzoneController(const AUVContext &cxt) : ControllerBase{cxt}
    {}

    bool dead_reckoning() override
    { return false; }

    void calc(const AUVContext &cxt, double dt, const FP &plan, const FP &estimate,
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

    explicit JerkController(const AUVContext &cxt) : ControllerBase{cxt}
    {}

    bool dead_reckoning() override
    { return false; }

    void calc(const AUVContext &cxt, double dt, const FP &plan, const FP &estimate,
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

    explicit BestController(const AUVContext &cxt) : ControllerBase{cxt}
    {}

    bool dead_reckoning() override
    { return false; }

    void calc(const AUVContext &cxt, double dt, const FP &plan, const FP &estimate,
              const orca::Acceleration &ff, orca::Acceleration &u_bar) override;
  };

  //=====================================================================================
  // DepthController controls depth, and ignores error on x, y and yaw
  //=====================================================================================

  class DepthController : public ControllerBase
  {
  public:

    explicit DepthController(const AUVContext &cxt) : ControllerBase{cxt}
    {}

    bool dead_reckoning() override
    { return true; }

    void calc(const AUVContext &cxt, double dt, const FP &plan, const FP &estimate,
              const orca::Acceleration &ff, orca::Acceleration &u_bar) override;
  };
#endif

  //=====================================================================================
  // Observation controller uses fiducial_vlam observations, not poses
  // Observations are in the body frame, not the world frame
  //=====================================================================================

  class ObservationController
  {
    const AUVContext &cxt_;

    // PID controllers
    pid::Controller forward_controller_;
    pid::Controller vertical_controller_;
    pid::Controller yaw_controller_;

  public:

    explicit ObservationController(const AUVContext &cxt);

    // The observation is pretty noisy for z, so pass in z data from the barometer
    void
    calc(const rclcpp::Duration &d, const Observation &plan, double plan_z, const Observation &estimate,
         double estimate_z, const orca::AccelerationBody &ff, orca::Efforts &efforts);
  };

} // namespace orca_base

#endif // ORCA_BASE_CONTROLLER_HPP