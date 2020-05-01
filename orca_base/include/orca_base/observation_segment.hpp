#ifndef ORCA_BASE_OBSERVATION_SEGMENT_HPP
#define ORCA_BASE_OBSERVATION_SEGMENT_HPP

#include "orca_base/segment_common.hpp"
#include "orca_shared/mw/mw.hpp"

namespace orca_base
{

  //=====================================================================================
  // Observation segments plan motion based on marker observations
  // Observations are in the camera_link frame, not the base_link frame
  // This means that the sub is ~20cm further away than obs.distance, and obs.bearing is exaggerated
  // In practice we can ignore this
  //=====================================================================================

  class ObservationSegmentBase : public SegmentBase
  {
  protected:

    mw::PolarObservationStamped plan_;    // Goal observation
    mw::PolarObservation goal_;           // Planned observation, incremented with each call to advance()

    mw::TwistBody twist_;           // Velocity in the body frame
    mw::AccelerationBody ff_;       // Acceleration in the body frame

  public:
    ObservationSegmentBase(const AUVContext &cxt, uint8_t type, mw::PolarObservationStamped start, mw::PolarObservation goal);

    const mw::PolarObservationStamped &plan() const
    { return plan_; }

    const mw::AccelerationBody &ff() const
    { return ff_; }
  };

  //=====================================================================================
  // RotateToMarker rotates to face a marker
  //=====================================================================================

  class RotateToMarker : public ObservationSegmentBase
  {
    mw::AccelerationBody initial_accel_;  // Initial total acceleration, not modified
    mw::AccelerationBody accel_;          // Total acceleration, accel_ = drag_ + ff_
    mw::AccelerationBody drag_;           // Acceleration due to drag

    // Start time
    rclcpp::Time start_{0, 0, RCL_ROS_TIME};

    // Time to change from one phase to another
    rclcpp::Time yaw_run_{0, 0, RCL_ROS_TIME};
    rclcpp::Time yaw_decel_{0, 0, RCL_ROS_TIME};
    rclcpp::Time yaw_stop_{0, 0, RCL_ROS_TIME};

  public:

    RotateToMarker(const AUVContext &cxt, const mw::PolarObservationStamped &start, const mw::PolarObservation &goal);

    std::string to_str() override;

    bool advance(const rclcpp::Duration &d) override;
  };

  //=====================================================================================
  // MoveToMarker moves forward toward a marker
  //=====================================================================================

  class MoveToMarker : public ObservationSegmentBase
  {
    mw::AccelerationBody initial_accel_;  // Initial total acceleration, not modified
    mw::AccelerationBody accel_;          // Total acceleration, accel_ = drag_ + ff_
    mw::AccelerationBody drag_;           // Acceleration due to drag

    // Start time
    rclcpp::Time start_{0, 0, RCL_ROS_TIME};

    // Time to change from one phase to another
    rclcpp::Time f_run_{0, 0, RCL_ROS_TIME};
    rclcpp::Time f_decel_{0, 0, RCL_ROS_TIME};
    rclcpp::Time f_stop_{0, 0, RCL_ROS_TIME};

  public:

    MoveToMarker(const AUVContext &cxt, const mw::PolarObservationStamped &start, const mw::PolarObservation &goal);

    std::string to_str() override;

    bool advance(const rclcpp::Duration &d) override;
  };

} // namespace orca_base

#endif //ORCA_BASE_OBSERVATION_SEGMENT_HPP