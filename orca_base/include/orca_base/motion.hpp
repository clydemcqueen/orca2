#ifndef ORCA_BASE_MOTION_HPP
#define ORCA_BASE_MOTION_HPP

#include "rclcpp/rclcpp.hpp"

#include "orca_base/base_context.hpp"
#include "orca_base/geometry.hpp"
#include "orca_base/pid.hpp"

namespace orca_base {

//=====================================================================================
// Constants
//=====================================================================================

constexpr double VELO_XY = 0.5;                 // Velocity for xy motion (m/s)
constexpr double VELO_Z = 0.3;                  // Velocity for z motion (m/s)
constexpr double EPSILON_PLAN_XYZ = 0.05;       // Close enough for xyz motion (m)
constexpr double VELO_YAW = M_PI / 10;          // Rotation velocity (r/s)
constexpr double EPSILON_PLAN_YAW = M_PI / 90;  // Close enough for yaw motion (r)

//=====================================================================================
// BaseMotion is a never-ending pid controller
// -- hold x, y, z, yaw at start value
//=====================================================================================

class BaseMotion
{
protected:

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

  BaseMotion(rclcpp::Logger &logger, const BaseContext &cxt, const Pose &start, const Pose &goal);

  // Advance the motion plan, return true to continue, false if we're done
  virtual bool advance(double dt, const Pose &estimate, Acceleration &u_bar);
};

//=====================================================================================
// VerticalMotion ascends or descends
// -- hold x, y, yaw at start value
//=====================================================================================

class VerticalMotion: public BaseMotion
{
public:

  VerticalMotion(rclcpp::Logger &logger, const BaseContext &cxt, const Pose &start, const Pose &goal);

  bool advance(double dt, const Pose &estimate, Acceleration &u_bar) override;
};

//=====================================================================================
// RotateMotion rotates about a point
// -- hold x, y, z at start value
//=====================================================================================

class RotateMotion: public BaseMotion
{
public:

  RotateMotion(rclcpp::Logger &logger, const BaseContext &cxt, const Pose &start, const Pose &goal);

  bool advance(double dt, const Pose &estimate, Acceleration &u_bar) override;
};

//=====================================================================================
// LineMotion moves in a straight line
// -- hold z, yaw at start value
//=====================================================================================

class LineMotion: public BaseMotion
{
public:

  LineMotion(rclcpp::Logger &logger, const BaseContext &cxt, const Pose &start, const Pose &goal);

  bool advance(double dt, const Pose &estimate, Acceleration &u_bar) override;
};

} // namespace orca_base

#endif // ORCA_BASE_MOTION_HPP