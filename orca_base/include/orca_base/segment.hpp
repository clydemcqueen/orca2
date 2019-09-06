#ifndef ORCA_BASE_SEGMENT_HPP
#define ORCA_BASE_SEGMENT_HPP

#include "rclcpp/rclcpp.hpp"

#include "orca_base/controller.hpp"

namespace orca_base
{

//=====================================================================================
// BaseSegment stays in one spot, holds x, y, z, yaw at start value
//=====================================================================================

  class BaseSegment
  {
  protected:

    rclcpp::Logger logger_;

    // State
    Pose goal_;       // Goal pose
    Pose plan_;       // Planned pose, incremented with each call to advance()
    Twist twist_;     // Velocity
    Acceleration ff_; // Acceleration

    void finish();

  public:

    BaseSegment(const rclcpp::Logger &logger, const BaseContext &cxt, const Pose &start, const Pose &goal);

    const Pose &plan() const
    { return plan_; }

    const Acceleration &ff() const
    { return ff_; }

    // Advance the motion plan, return true to continue, false if we're done
    // TODO use std::chrono::milliseconds instead of double
    virtual bool advance(double dt);
  };

//=====================================================================================
// VerticalSegment ascends or descends, holds x, y, yaw at start value
//=====================================================================================

  class VerticalSegment : public BaseSegment
  {
  public:

    VerticalSegment(const rclcpp::Logger &logger, const BaseContext &cxt, const Pose &start, const Pose &goal);

    bool advance(double dt) override;
  };

//=====================================================================================
// RotateSegment rotates about a point, holds x, y, z at start value
//=====================================================================================

  class RotateSegment : public BaseSegment
  {
  public:

    RotateSegment(const rclcpp::Logger &logger, const BaseContext &cxt, const Pose &start, const Pose &goal);

    bool advance(double dt) override;
  };

//=====================================================================================
// LineSegment moves in a straight line, holds z, yaw at start value
//=====================================================================================

  class LineSegment : public BaseSegment
  {
  public:

    LineSegment(const rclcpp::Logger &logger, const BaseContext &cxt, const Pose &start, const Pose &goal);

    bool advance(double dt) override;
  };

} // namespace orca_base

#endif //ORCA_BASE_SEGMENT_HPP