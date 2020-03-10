#ifndef ORCA_BASE_SEGMENT_HPP
#define ORCA_BASE_SEGMENT_HPP

#include "orca_base/auv_context.hpp"
#include "orca_base/fp.hpp"

namespace orca_base
{

  //=====================================================================================
  // Segments describe a trajectory from start to goal over time
  //=====================================================================================

  class SegmentBase
  {
  protected:

    AUVContext cxt_;
    uint8_t type_;

  public:

    explicit SegmentBase(AUVContext cxt, uint8_t type);

    // Return a string suitable for logging
    virtual std::string to_str() = 0;

    // Advance the motion plan by dt seconds, return true to continue, false if we're done
    virtual bool advance(const rclcpp::Duration &d) = 0;

    __uint8_t type()
    { return type_; }

    std::string type_name();
  };

  //=====================================================================================
  // Pose segments plan motion based on poses
  //=====================================================================================

  class PoseSegmentBase : public SegmentBase
  {
  protected:

    FPStamped plan_;        // Planned pose, incremented with each call to advance()
    FP goal_;               // Goal pose

    orca::Twist twist_;     // Velocity in the world frame
    orca::Acceleration ff_; // Acceleration in the world frame

  public:
    PoseSegmentBase(const AUVContext &cxt, uint8_t type, FPStamped start, FP goal);

    const FPStamped &plan() const
    { return plan_; }

    const FP &goal() const
    { return goal_; }

    const orca::Twist &twist() const
    { return twist_; }

    const orca::Acceleration &ff() const
    { return ff_; }
  };

  //=====================================================================================
  // Observation segments plan motion based on marker observations
  // Observations are in the camera_link frame, not the base_link frame
  // This means that the sub is ~20cm further away than obs.distance, and obs.yaw is exaggerated
  // In practice we can ignore this
  //=====================================================================================

  class ObservationSegmentBase : public SegmentBase
  {
  protected:

    ObservationStamped plan_;   // Goal observation
    Observation goal_;          // Planned observation, incremented with each call to advance()

    orca::TwistBody twist_;     // Velocity in the body frame
    orca::AccelerationBody ff_; // Acceleration in the body frame

  public:
    ObservationSegmentBase(const AUVContext &cxt, uint8_t type, ObservationStamped start, Observation goal);

    const ObservationStamped &plan() const
    { return plan_; }

    const orca::TwistBody &twist() const
    { return twist_; }

    const orca::AccelerationBody &ff() const
    { return ff_; }
  };

  //=====================================================================================
  // Pause segments stay in one pose for a period of time
  //=====================================================================================

  class Pause : public PoseSegmentBase
  {
    rclcpp::Duration pause_duration_;     // Time remaining

  public:

    Pause(const AUVContext &cxt, const FPStamped &start, const rclcpp::Duration &pause_duration);

    std::string to_str() override;

    bool advance(const rclcpp::Duration &d) override;
  };

  //=====================================================================================
  // RotateToMarker rotates to face a marker
  //=====================================================================================

  class RotateToMarker : public ObservationSegmentBase
  {
    orca::AccelerationBody initial_accel_;  // Initial total acceleration, not modified
    orca::AccelerationBody accel_;          // Total acceleration, accel_ = drag_ + ff_
    orca::AccelerationBody drag_;           // Acceleration due to drag

    // Start time
    rclcpp::Time start_{0, 0, RCL_ROS_TIME};

    // Time to change from one phase to another
    rclcpp::Time yaw_run_{0, 0, RCL_ROS_TIME};
    rclcpp::Time yaw_decel_{0, 0, RCL_ROS_TIME};
    rclcpp::Time yaw_stop_{0, 0, RCL_ROS_TIME};

  public:

    RotateToMarker(const AUVContext &cxt, const ObservationStamped &start, const Observation &goal);

    std::string to_str() override;

    bool advance(const rclcpp::Duration &d) override;
  };

  //=====================================================================================
  // MoveToMarker moves forward toward a marker
  //=====================================================================================

  class MoveToMarker : public ObservationSegmentBase
  {
    orca::AccelerationBody initial_accel_;  // Initial total acceleration, not modified
    orca::AccelerationBody accel_;          // Total acceleration, accel_ = drag_ + ff_
    orca::AccelerationBody drag_;           // Acceleration due to drag

    // Start time
    rclcpp::Time start_{0, 0, RCL_ROS_TIME};

    // Time to change from one phase to another
    rclcpp::Time f_run_{0, 0, RCL_ROS_TIME};
    rclcpp::Time f_decel_{0, 0, RCL_ROS_TIME};
    rclcpp::Time f_stop_{0, 0, RCL_ROS_TIME};

  public:

    MoveToMarker(const AUVContext &cxt, const ObservationStamped &start, const Observation &goal);

    std::string to_str() override;

    bool advance(const rclcpp::Duration &d) override;
  };

} // namespace orca_base

#endif //ORCA_BASE_SEGMENT_HPP