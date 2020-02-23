#ifndef ORCA_BASE_SEGMENT_HPP
#define ORCA_BASE_SEGMENT_HPP

#include "rclcpp/rclcpp.hpp"

#include "orca_base/base_context.hpp"
#include "orca_base/fp.hpp"

namespace orca_base
{

  //=====================================================================================
  // Segments describe a trajectory from start to goal over time
  //=====================================================================================

  class SegmentBase
  {
  protected:

    rclcpp::Logger logger_;
    BaseContext cxt_;

  public:

    SegmentBase(const rclcpp::Logger &logger, BaseContext cxt);

    // Write contents to RCLCPP_INFO
    virtual void log_info() = 0;

    // Advance the motion plan by dt seconds, return true to continue, false if we're done
    virtual bool advance(const rclcpp::Duration &d) = 0;
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
    PoseSegmentBase(const rclcpp::Logger &logger, const BaseContext &cxt, FPStamped start, FP goal);

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
    ObservationSegmentBase(const rclcpp::Logger &logger, const BaseContext &cxt, ObservationStamped start,
                           Observation goal);

    const ObservationStamped &plan() const
    { return plan_; }

    const Observation &goal() const
    { return goal_; }

    const orca::TwistBody &twist() const
    { return twist_; }

    const orca::AccelerationBody &ff() const
    { return ff_; }
  };

  //=====================================================================================
  // Use a trapezoidal velocity planner to plan motion in x, y, z, yaw
  // There are 3 velocity trapezoids: xy motion, z motion and yaw motion
  //=====================================================================================

  class TrapVelo : public PoseSegmentBase
  {
    double angle_to_goal_;

    orca::Acceleration initial_accel_;  // Initial total acceleration, not modified
    orca::Acceleration accel_;          // Total acceleration, accel_ = drag_ + ff_
    orca::Acceleration drag_;           // Acceleration due to drag
    orca::Twist twist_;                 // Velocity

    // Start time
    rclcpp::Time start_{0, 0, RCL_ROS_TIME};

    // Time to change from one phase to another
    rclcpp::Time xy_run_{0, 0, RCL_ROS_TIME};
    rclcpp::Time xy_decel_{0, 0, RCL_ROS_TIME};
    rclcpp::Time xy_stop_{0, 0, RCL_ROS_TIME};

    rclcpp::Time z_run_{0, 0, RCL_ROS_TIME};
    rclcpp::Time z_decel_{0, 0, RCL_ROS_TIME};
    rclcpp::Time z_stop_{0, 0, RCL_ROS_TIME};

    rclcpp::Time yaw_run_{0, 0, RCL_ROS_TIME};
    rclcpp::Time yaw_decel_{0, 0, RCL_ROS_TIME};
    rclcpp::Time yaw_stop_{0, 0, RCL_ROS_TIME};

  public:

    explicit TrapVelo(const rclcpp::Logger &logger, const BaseContext &cxt, const FPStamped &start, const FP &goal);

    // Return time required to complete all motion
    rclcpp::Duration duration() const;

    void log_info() override;

    bool advance(const rclcpp::Duration &d) override;

    // Factory methods: make a segment that gets from plan to goal, and update plan
    static std::shared_ptr<TrapVelo>
    make_vertical(const rclcpp::Logger &logger, const BaseContext &cxt, FPStamped &plan, double z);

    static std::shared_ptr<TrapVelo>
    make_rotate(const rclcpp::Logger &logger, const BaseContext &cxt, FPStamped &plan, double yaw);

    static std::shared_ptr<TrapVelo>
    make_line(const rclcpp::Logger &logger, const BaseContext &cxt, FPStamped &plan, double x, double y);

    static std::shared_ptr<TrapVelo>
    make_pose(const rclcpp::Logger &logger, const BaseContext &cxt, FPStamped &plan, const FP &goal);
  };

  //=====================================================================================
  // Pause segments stay in one pose for a period of time
  //=====================================================================================

  class Pause : public PoseSegmentBase
  {
    rclcpp::Duration d_;    // Time remaining

  public:

    Pause(const rclcpp::Logger &logger, const BaseContext &cxt, const FPStamped &start, const rclcpp::Duration &d);

    void log_info() override;

    const rclcpp::Duration &time_remaining() const
    { return d_; }

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

    RotateToMarker(const rclcpp::Logger &logger, const BaseContext &cxt, const ObservationStamped &start,
                   const Observation &goal);

    void log_info() override;

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

    MoveToMarker(const rclcpp::Logger &logger, const BaseContext &cxt, const ObservationStamped &start,
                 const Observation &goal);

    void log_info() override;

    bool advance(const rclcpp::Duration &d) override;
  };

} // namespace orca_base

#endif //ORCA_BASE_SEGMENT_HPP