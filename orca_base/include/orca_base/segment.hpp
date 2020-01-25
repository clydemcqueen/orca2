#ifndef ORCA_BASE_SEGMENT_HPP
#define ORCA_BASE_SEGMENT_HPP

#include "rclcpp/rclcpp.hpp"

#include "orca_base/controller.hpp"

namespace orca_base
{

  //=====================================================================================
  // Constants
  //=====================================================================================

  constexpr double EPSILON_PLAN_XYZ = 0.05;       // Close enough for xyz motion (m)
  constexpr double EPSILON_PLAN_YAW = M_PI / 90;  // Close enough for yaw motion (r)

  //=====================================================================================
  // Segments describe a trajectory from start to goal over time
  //=====================================================================================

  class SegmentBase
  {
  protected:

    rclcpp::Logger logger_;
    BaseContext cxt_;

    // State
    orca::FP goal_;       // Goal pose
    orca::FP plan_;       // Planned pose, incremented with each call to advance()

//    void finish();

  public:

    SegmentBase(const rclcpp::Logger &logger, const BaseContext &cxt, orca::FP start, orca::FP goal);

    // Write contents to RCLCPP_INFO
    virtual void log_info() = 0;

    // Return the current (planned) pose
    const orca::FP &plan() const
    { return plan_; }

    // Return the goal pose
    const orca::FP &goal() const
    { return goal_; }

    // Return the acceleration required at the moment
    virtual const orca::Acceleration &ff() const = 0;
//    { return ff_; }

    // Advance the motion plan by dt seconds, return true to continue, false if we're done
    virtual bool advance(double dt) = 0;
  };

  //=====================================================================================
  // Pose segments plan motion based on poses
  //=====================================================================================

  class PoseSegmentBase : public SegmentBase
  {
  protected:

    orca::Twist twist_;     // Velocity in the world frame
    orca::Acceleration ff_; // Acceleration in the world frame

  public:
    PoseSegmentBase(const rclcpp::Logger &logger, const BaseContext &cxt, orca::FP start, orca::FP goal);

    const orca::Acceleration &ff() const override
    { return ff_; }

    // TODO advance() should project pose to observation
  };

  //=====================================================================================
  // Observation segments plan motion based on observations
  //=====================================================================================

  class ObservationSegmentBase : public SegmentBase
  {
  protected:

    orca::Twist twist_body_;     // Velocity in the body frame
    orca::Acceleration ff_body_; // Acceleration in the body frame

  public:
    ObservationSegmentBase(const rclcpp::Logger &logger, const BaseContext &cxt, orca::FP start, orca::FP goal);

    // TODO transform body to world???
    const orca::Acceleration &ff() const override
    { return ff_body_; }
  };

  //=====================================================================================
  // Pause stays in one spot for a period of time
  //=====================================================================================

  class Pause : public PoseSegmentBase
  {
    double seconds_;

  public:

    Pause(const rclcpp::Logger &logger, const BaseContext &cxt, const orca::FP &start, double seconds);

    void log_info() override;

    bool advance(double dt) override;
  };

  //=====================================================================================
  // VerticalSegment ascends or descends, holds x, y, yaw at start value
  //=====================================================================================

  class VerticalSegment : public PoseSegmentBase
  {
  public:

    VerticalSegment(const rclcpp::Logger &logger, const BaseContext &cxt, const orca::FP &start, const orca::FP &goal);

    void log_info() override;

    bool advance(double dt) override;
  };

  //=====================================================================================
  // RotateSegment rotates about a point, holds x, y, z at start value
  //=====================================================================================

  class RotateSegment : public PoseSegmentBase
  {
  public:

    RotateSegment(const rclcpp::Logger &logger, const BaseContext &cxt, const orca::FP &start, const orca::FP &goal);

    void log_info() override;

    bool advance(double dt) override;
  };

  //=====================================================================================
  // LineSegment moves in a straight line, holds z, yaw at start value
  //=====================================================================================

  class LineSegment : public PoseSegmentBase
  {
  public:

    LineSegment(const rclcpp::Logger &logger, const BaseContext &cxt, const orca::FP &start, const orca::FP &goal);

    void log_info() override;

    bool advance(double dt) override;
  };

  //=====================================================================================
  // MoveToMarkerSegment uses observations, not poses
  //=====================================================================================

  class MoveToMarkerSegment : ObservationSegmentBase
  {
  public:

    MoveToMarkerSegment(const rclcpp::Logger &logger, const BaseContext &cxt, const orca::FP &start,
                        const orca::FP &goal);

    void log_info() override;

    bool advance(double dt) override;
  };

} // namespace orca_base

#endif //ORCA_BASE_SEGMENT_HPP