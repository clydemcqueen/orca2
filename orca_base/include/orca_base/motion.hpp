#ifndef ORCA_BASE_MOTION_HPP
#define ORCA_BASE_MOTION_HPP

#include "eigen3/Eigen/Geometry"
#include "orca_base/model.hpp"
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

  rclcpp::Logger logger_;

  // Goal state
  OrcaPose goal_;

  // Feedforward = planned acceleration + acceleration due to drag
  OrcaPose ff_;

  // PID controllers
  std::shared_ptr<pid::Controller> x_controller_;
  std::shared_ptr<pid::Controller> y_controller_;
  std::shared_ptr<pid::Controller> z_controller_;
  std::shared_ptr<pid::Controller> yaw_controller_;

public:

  BaseMotion(rclcpp::Logger logger): logger_{logger} {}

  // Initialize the motion plan, return true if successful
  virtual bool init(const OrcaPose &goal, OrcaOdometry &plan);

  // Advance the motion plan, return true to continue, false if we're done
  virtual bool advance(double dt, const OrcaPose &curr, OrcaOdometry &plan, OrcaPose &u_bar);
};

//=====================================================================================
// RotateMotion rotates about a point
// -- hold x, y, z at start value
//
// Limitations:
// -- assume instant thrust force
// -- assume instant acceleration
//=====================================================================================

class RotateMotion: public BaseMotion
{
public:

  RotateMotion(rclcpp::Logger logger): BaseMotion{logger} {}

  bool init(const OrcaPose &goal, OrcaOdometry &plan) override;
  bool advance(double dt, const OrcaPose &curr, OrcaOdometry &plan, OrcaPose &u_bar) override;
};

//=====================================================================================
// LineMotion moves in a straight line
// -- hold z, yaw at start value
//
// Phase 1: apply a constant thrust; asymptotically hit desired velocity
// Phase 2: cut thrusters and decelerate
//
// Limitations:
// -- assume instant thrust force
//=====================================================================================

class LineMotion: public BaseMotion
{
public:

  LineMotion(rclcpp::Logger logger): BaseMotion{logger} {}

  bool init(const OrcaPose &goal, OrcaOdometry &plan) override;
  bool advance(double dt, const OrcaPose &curr, OrcaOdometry &plan, OrcaPose &u_bar) override;
};

//=====================================================================================
// ArcMotion moves in an arc
// -- radius is computed as the distance from start to goal
// -- yaw will always point in the direction of motion
// -- hold z at start value
//
// Limitations:
// -- center is always to the left of the start position, always moves ccw
// -- assume instant thrust force
// -- assume instant acceleration
//=====================================================================================

class ArcMotion: public BaseMotion
{
private:

  // Simple arc
  struct Arc
  {
    // Simple 2d point
    struct Point
    {
      double x;
      double y;
    };

    // ENU polar angle, 0 = East, increment counter-clockwise, in radians
    typedef double Polar;

    Point center_;
    double radius_;
    Polar start_angle_;   // Arc start angle
    Polar goal_angle_;    // Arc goal angle

    // Convert a polar angle on this arc to an OrcaPose
    void polar_to_cartesian(const Polar theta, OrcaPose &pose) const
    {
      pose.x = center_.x + radius_ * cos(theta);
      pose.y = center_.y + radius_ * sin(theta);
      pose.yaw = norm_angle(theta + M_PI_2);
    }

    // Convert an OrcaPose to a polar angle on this arc
    void cartesian_to_polar(const OrcaPose &pose, Polar &theta) const
    {
      // TODO assert that pose is on the circle
      theta = atan2(pose.y - center_.y, pose.x - center_.x);
    }
  };

  // Planned motion in polar coordinates
  Arc arc_;
  Arc::Polar polar_pose_;
  Arc::Polar polar_velo_;

public:

  ArcMotion(rclcpp::Logger logger): BaseMotion{logger} {}

  bool init(const OrcaPose &goal, OrcaOdometry &plan) override;
  bool advance(double dt, const OrcaPose &curr, OrcaOdometry &plan, OrcaPose &u_bar) override;
};

//=====================================================================================
// VerticalMotion ascends or descends
// -- hold x, y, yaw at start value
//
// Limitations:
// -- assume instant thrust force
// -- assume instant acceleration
//=====================================================================================

class VerticalMotion: public BaseMotion
{
public:

  VerticalMotion(rclcpp::Logger logger): BaseMotion{logger} {}

  bool init(const OrcaPose &goal, OrcaOdometry &plan) override;
  bool advance(double dt, const OrcaPose &curr, OrcaOdometry &plan, OrcaPose &u_bar) override;
};

} // namespace orca_base

#endif // ORCA_BASE_MOTION_HPP