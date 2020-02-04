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

  public:

    SegmentBase(const rclcpp::Logger &logger, BaseContext cxt);

    // Write contents to RCLCPP_INFO
    virtual void log_info() = 0;

    // Advance the motion plan by dt seconds, return true to continue, false if we're done
    virtual bool advance(double dt) = 0;
  };

  //=====================================================================================
  // Pose segments plan motion based on poses
  //=====================================================================================

  class PoseSegmentBase : public SegmentBase
  {
  protected:

    orca::FP plan_;         // Planned pose, incremented with each call to advance()
    orca::FP goal_;         // Goal pose

    orca::Twist twist_;     // Velocity in the world frame
    orca::Acceleration ff_; // Acceleration in the world frame

  public:
    PoseSegmentBase(const rclcpp::Logger &logger, const BaseContext &cxt, orca::FP start, orca::FP goal);

    const orca::FP &plan() const
    { return plan_; }

    const orca::FP &goal() const
    { return goal_; }

    const orca::Twist &twist() const
    { return twist_; }

    const orca::Acceleration &ff() const
    { return ff_; }
  };

  //=====================================================================================
  // Observation segments plan motion based on observations
  //=====================================================================================

  class ObservationSegmentBase : public SegmentBase
  {
  protected:

    orca::Observation plan_;    // Goal observation
    orca::Observation goal_;    // Planned observation, incremented with each call to advance()

    orca::TwistBody twist_;     // Velocity in the body frame
    orca::AccelerationBody ff_; // Acceleration in the body frame

  public:
    ObservationSegmentBase(const rclcpp::Logger &logger, const BaseContext &cxt, orca::Observation start,
                           orca::Observation goal);

    const orca::Observation &plan() const
    { return plan_; }

    const orca::Observation &goal() const
    { return goal_; }

    const orca::TwistBody &twist() const
    { return twist_; }

    const orca::AccelerationBody &ff() const
    { return ff_; }
  };

  //=====================================================================================
  // Pose segments can move in x, y, z, yaw, or any of those in combination
  //=====================================================================================

  class PoseSegment : public PoseSegmentBase
  {
    orca::Twist target_twist_;

  public:

    PoseSegment(const rclcpp::Logger &logger, const BaseContext &cxt, const orca::FP &start, const orca::FP &goal);

    void log_info() override;

    bool advance(double dt) override;

    // Factory methods: make a segment that gets from plan to goal, and update plan
    static std::shared_ptr<PoseSegment>
    make_vertical(const rclcpp::Logger &logger, const BaseContext &cxt, orca::FP &plan, double z);

    static std::shared_ptr<PoseSegment>
    make_rotate(const rclcpp::Logger &logger, const BaseContext &cxt, orca::FP &plan, double yaw);

    static std::shared_ptr<PoseSegment>
    make_line(const rclcpp::Logger &logger, const BaseContext &cxt, orca::FP &plan, double x, double y);
  };


  //=====================================================================================
  // Pause segments stay in one pose for a period of time
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
  // RotateToMarkerSegment rotates to face a marker
  //=====================================================================================

  class RotateToMarkerSegment : public ObservationSegmentBase
  {
  public:

    RotateToMarkerSegment(const rclcpp::Logger &logger, const BaseContext &cxt, const orca::Observation &start,
                          const orca::Observation &goal);

    void log_info() override;

    bool advance(double dt) override;
  };

  //=====================================================================================
  // MoveToMarkerSegment moves forward toward a marker
  //=====================================================================================

  class MoveToMarkerSegment : public ObservationSegmentBase
  {
  public:

    MoveToMarkerSegment(const rclcpp::Logger &logger, const BaseContext &cxt, const orca::Observation &start,
                        const orca::Observation &goal);

    void log_info() override;

    bool advance(double dt) override;
  };

} // namespace orca_base

#endif //ORCA_BASE_SEGMENT_HPP