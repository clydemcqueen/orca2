#ifndef ORCA_BASE_CONTROLLER_HPP
#define ORCA_BASE_CONTROLLER_HPP

#include "orca_base/auv_context.hpp"
#include "orca_base/pid.hpp"
#include "orca_shared/mw/mw.hpp"

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

    void calc(const rclcpp::Duration &d, const mw::FiducialPose &plan, const mw::FiducialPose &estimate, const mw::Acceleration &ff,
              mw::Efforts &efforts);
  };

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
    pid::Controller bearing_controller_;

  public:

    explicit ObservationController(const AUVContext &cxt);

    // The observation is pretty noisy for z, so pass in z data from the barometer
    void
    calc(const rclcpp::Duration &d, const mw::PolarObservation &plan, double plan_z, const mw::PolarObservation &estimate,
         double estimate_z, const mw::AccelerationBody &ff, mw::Efforts &efforts);
  };

} // namespace orca_base

#endif // ORCA_BASE_CONTROLLER_HPP