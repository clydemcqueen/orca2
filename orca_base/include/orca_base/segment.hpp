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
    orca::Pose goal_;       // Goal pose
    orca::Pose plan_;       // Planned pose, incremented with each call to advance()
    orca::Twist twist_;     // Velocity
    orca::Acceleration ff_; // Acceleration

    void finish();

  public:

    SegmentBase(const rclcpp::Logger &logger, const BaseContext &cxt, const orca::Pose &start, const orca::Pose &goal);

    // Write contents to RCLCPP_INFO
    virtual void log_info() = 0;

    // Try to change the goal, return true if it worked
    virtual bool extend(const orca::Pose &start, const orca::Pose &goal)
    { return false; }

    // Return the current (planned) pose
    const orca::Pose &plan() const
    { return plan_; }

    // Return the goal pose
    const orca::Pose &goal() const
    { return goal_; }

    // Return the acceleration required at the moment
    const orca::Acceleration &ff() const
    { return ff_; }

    // Advance the motion plan by dt seconds, return true to continue, false if we're done
    virtual bool advance(double dt) = 0;
  };

  //=====================================================================================
  // Pause stays in one spot for a period of time
  //=====================================================================================

  class Pause : public SegmentBase
  {
    double seconds_;

  public:

    Pause(const rclcpp::Logger &logger, const BaseContext &cxt, const orca::Pose &start, double seconds);

    void log_info() override;

    bool advance(double dt) override;
  };

  //=====================================================================================
  // VerticalSegment ascends or descends, holds x, y, yaw at start value
  //=====================================================================================

  class VerticalSegment : public SegmentBase
  {
  public:

    VerticalSegment(const rclcpp::Logger &logger, const BaseContext &cxt, const orca::Pose &start,
                    const orca::Pose &goal);

    void log_info() override;

    bool advance(double dt) override;
  };

  //=====================================================================================
  // RotateSegment rotates about a point, holds x, y, z at start value
  //=====================================================================================

  class RotateSegment : public SegmentBase
  {
  public:

    RotateSegment(const rclcpp::Logger &logger, const BaseContext &cxt, const orca::Pose &start,
                  const orca::Pose &goal);

    void log_info() override;

    bool advance(double dt) override;
  };

  //=====================================================================================
  // LineSegment moves in a straight line, holds z, yaw at start value
  //=====================================================================================

  class LineSegment : public SegmentBase
  {
    void init();

  public:

    LineSegment(const rclcpp::Logger &logger, const BaseContext &cxt, const orca::Pose &start, const orca::Pose &goal);

    void log_info() override;

    bool extend(const orca::Pose &start, const orca::Pose &goal) override;

    bool advance(double dt) override;
  };

  //=====================================================================================
  // MoveToMarkerSegment uses observations, not poses
  //=====================================================================================

  class MoveToMarkerSegment
  {
    rclcpp::Logger logger_;
    BaseContext cxt_;

    // State
    orca::FiducialPoseStamped goal_;    // Goal pose
    orca::FiducialPoseStamped plan_;    // Planned pose, incremented with each call to advance()
    orca::Twist twist_;         // Velocity in the body frame
    orca::Acceleration ff_;     // Acceleration in the body frame

  public:

    MoveToMarkerSegment(const rclcpp::Logger &logger, BaseContext cxt, orca::FiducialPoseStamped start, orca::FiducialPoseStamped goal);

    // Return the current (planned) pose
    const orca::FiducialPoseStamped &plan() const
    { return plan_; }

    // Return the goal pose
    const orca::FiducialPoseStamped &goal() const
    { return goal_; }

    // Return the acceleration required at the moment -- in the body frame!
    const orca::Acceleration &ff() const
    { return ff_; }

    bool advance(double dt);
  };

} // namespace orca_base

#endif //ORCA_BASE_SEGMENT_HPP