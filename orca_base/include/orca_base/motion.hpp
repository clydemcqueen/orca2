#ifndef ORCA_BASE_MOTION_HPP
#define ORCA_BASE_MOTION_HPP

#include "rclcpp/rclcpp.hpp"

#include "orca_base/base_context.hpp"
#include "orca_base/geometry.hpp"
#include "orca_base/pid.hpp"

namespace orca_base
{

//=====================================================================================
// BaseMotion is a never-ending pid controller
// -- hold x, y, z, yaw at start value
//=====================================================================================

  class BaseMotion
  {
  protected:

    rclcpp::Logger logger_;

    // State
    Pose goal_;       // Goal pose
    Pose plan_;       // Planned pose, incremented with each call to advance()
    Twist twist_;     // Velocity
    Acceleration ff_; // Acceleration

    // PID controllers
    pid::Controller x_controller_;
    pid::Controller y_controller_;
    pid::Controller z_controller_;
    pid::Controller yaw_controller_;

    void finish(Acceleration &u_bar);

  public:

    BaseMotion(const rclcpp::Logger &logger, const BaseContext &cxt, const Pose &start, const Pose &goal);

    // Advance the motion plan, return true to continue, false if we're done
    // TODO use std::chrono::milliseconds instead of double
    virtual bool advance(double dt, const Pose &estimate, Acceleration &u_bar, PoseError &error);
  };

//=====================================================================================
// VerticalMotion ascends or descends
// -- hold x, y, yaw at start value
//=====================================================================================

  class VerticalMotion : public BaseMotion
  {
  public:

    VerticalMotion(const rclcpp::Logger &logger, const BaseContext &cxt, const Pose &start, const Pose &goal);

    bool advance(double dt, const Pose &estimate, Acceleration &u_bar, PoseError &error) override;
  };

//=====================================================================================
// RotateMotion rotates about a point
// -- hold x, y, z at start value
//=====================================================================================

  class RotateMotion : public BaseMotion
  {
  public:

    RotateMotion(const rclcpp::Logger &logger, const BaseContext &cxt, const Pose &start, const Pose &goal);

    bool advance(double dt, const Pose &estimate, Acceleration &u_bar, PoseError &error) override;
  };

//=====================================================================================
// LineMotion moves in a straight line
// -- hold z, yaw at start value
//=====================================================================================

  class LineMotion : public BaseMotion
  {
  public:

    LineMotion(const rclcpp::Logger &logger, const BaseContext &cxt, const Pose &start, const Pose &goal);

    bool advance(double dt, const Pose &estimate, Acceleration &u_bar, PoseError &error) override;
  };

} // namespace orca_base

#endif // ORCA_BASE_MOTION_HPP