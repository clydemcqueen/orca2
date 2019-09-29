#ifndef ORCA_BASE_MODEL_HPP
#define ORCA_BASE_MODEL_HPP

#include <cmath>

namespace orca_base
{

  // TODO move these to a struct so we can override physics and other constants from parameters

  //=====================================================================================
  // Physics
  //=====================================================================================

  constexpr double GRAVITY = 9.8;                   // Gazebo default is 9.8, 9.80665 is a bit more accurate
  constexpr double FLUID_DENSITY = 1029;            // Fluid density of seawater TODO cxt should specify fresh vs. seawater
  constexpr double ATMOSPHERIC_PRESSURE = 101300;   // Air pressure at the surface

  constexpr double depth_z(double pressure)
  { return -(pressure - ATMOSPHERIC_PRESSURE) / (FLUID_DENSITY * GRAVITY); }

  //=====================================================================================
  // Sensor constants
  //=====================================================================================

  constexpr double DEPTH_STDDEV = 0.1;

  //=====================================================================================
  // Vehicle specs, in the body frame (x forward, y left, z up)
  //=====================================================================================

  constexpr double ROV_DIM_X = 0.457;       // Length
  constexpr double ROV_DIM_Y = 0.338;       // Width
  constexpr double ROV_DIM_Z = 0.254;       // Height

  constexpr double TETHER_DIAM = 0.008;

  constexpr double ROV_AREA_X = ROV_DIM_Y * ROV_DIM_Z;  // Area of front (bow) and rear (stern) sides
  constexpr double ROV_AREA_Y = ROV_DIM_X * ROV_DIM_Z;  // Area of top and bottom
  constexpr double ROV_AREA_Z = ROV_DIM_X * ROV_DIM_Y;  // Area of left (port) and right (starboard) sides

  constexpr double ORCA_MASS = 10;
  constexpr double ORCA_VOLUME = 0.01;

  // Assume a uniform distribution of mass in the vehicle box
  constexpr double MOMENT_OF_INERTIA_YAW = ORCA_MASS / 12.0 * (ROV_DIM_X * ROV_DIM_X + ROV_DIM_Y * ROV_DIM_Y);

  // From BlueRobotics specs, all forces are in Newtons
  constexpr double BOLLARD_FORCE_XY = 137;
  constexpr double BOLLARD_FORCE_Z = 88;
  constexpr double T200_MAX_POS_FORCE = 50;
  constexpr double T200_MAX_NEG_FORCE = 40;

  // Estimate maximum yaw torque by looking at 4 thrusters (2 forward, 2 reverse), each mounted ~tangent to a circle with radius = 18cm
  constexpr double MAX_TORQUE_YAW = 0.18 * 2.0 * (T200_MAX_POS_FORCE + T200_MAX_NEG_FORCE);

  constexpr double DISPLACED_MASS = ORCA_VOLUME * FLUID_DENSITY;
  constexpr double WEIGHT_IN_WATER = GRAVITY * (ORCA_MASS - DISPLACED_MASS);
  constexpr double HOVER_ACCEL_Z = WEIGHT_IN_WATER / ORCA_MASS; // Z acceleration required to hover

  //=====================================================================================
  // Drag constants, in the body frame (x forward, y left, z up)
  //
  // drag = 0.5 * density * area * velocity^2 * coefficient
  //    the drag coefficient for a box is 1.0
  //    the drag coefficient for an unfaired tether is 1.2
  //
  // The ROV constants below capture all but velocity:
  //    constant = 0.5 * density * area * coefficient
  //
  // The tether constant below captures all but depth and velocity:
  //    constant = 0.5 * density * width * coefficient
  //=====================================================================================

  constexpr double DRAG_COEFFICIENT_X = 0.8;    // Estimated
  constexpr double DRAG_COEFFICIENT_Y = 0.95;   // Estimated
  constexpr double DRAG_COEFFICIENT_Z = 0.95;   // Estimated

  constexpr double LINEAR_DRAG_X = 0.5 * FLUID_DENSITY * ROV_AREA_X * DRAG_COEFFICIENT_X;
  constexpr double LINEAR_DRAG_Y = 0.5 * FLUID_DENSITY * ROV_AREA_Y * DRAG_COEFFICIENT_Y;
  constexpr double LINEAR_DRAG_Z = 0.5 * FLUID_DENSITY * ROV_AREA_Z * DRAG_COEFFICIENT_Z;

  constexpr double ANGULAR_DRAG_YAW = 0.02 * (LINEAR_DRAG_X + LINEAR_DRAG_Y);   // Estimated

  constexpr double TETHER_DRAG_COEFFICIENT = 1.1;   // Estimated
  constexpr double TETHER_DRAG = 0.5 * FLUID_DENSITY * TETHER_DIAM * TETHER_DRAG_COEFFICIENT;

  //=====================================================================================
  // Dynamics, in the world frame (ENU)
  // 4 DoF, pitch and roll are 0
  //=====================================================================================

  // Velocity => drag force / torque
  constexpr double drag_force_x(double velo_x)
  { return velo_x * std::abs(velo_x) * -LINEAR_DRAG_X; }

  constexpr double drag_force_y(double velo_y)
  { return velo_y * std::abs(velo_y) * -LINEAR_DRAG_Y; }

  constexpr double drag_force_z(double velo_z)
  { return velo_z * std::abs(velo_z) * -LINEAR_DRAG_Z; }

  constexpr double drag_torque_yaw(double velo_yaw)
  { return velo_yaw * std::abs(velo_yaw) * -ANGULAR_DRAG_YAW; }

  // Force / torque => acceleration
  constexpr double force_to_accel(double force)
  { return force / ORCA_MASS; }

  constexpr double torque_to_accel_yaw(double torque_yaw)
  { return torque_yaw / MOMENT_OF_INERTIA_YAW; }

  // Maximum acceleration due to thrust
  constexpr double MAX_ACCEL_XY = force_to_accel(BOLLARD_FORCE_XY);
  constexpr double MAX_ACCEL_Z = force_to_accel(BOLLARD_FORCE_Z);
  constexpr double MAX_ACCEL_YAW = torque_to_accel_yaw(MAX_TORQUE_YAW);

  // Velocity => acceleration due to drag
  constexpr double drag_accel_x(double velo_x)
  { return force_to_accel(drag_force_x(velo_x)); }

  constexpr double drag_accel_y(double velo_y)
  { return force_to_accel(drag_force_y(velo_y)); }

  constexpr double drag_accel_z(double velo_z)
  { return force_to_accel(drag_force_z(velo_z)); }

  constexpr double drag_accel_yaw(double velo_yaw)
  { return torque_to_accel_yaw(drag_torque_yaw(velo_yaw)); }

  // Force / torque => effort
  constexpr double force_to_effort_xy(double force_xy)
  { return force_xy / BOLLARD_FORCE_XY; }

  constexpr double force_to_effort_z(double force_z)
  { return force_z / BOLLARD_FORCE_Z; }

  constexpr double torque_to_effort_yaw(double torque_yaw)
  { return torque_yaw / MAX_TORQUE_YAW; }

  // Acceleration => force / torque
  constexpr double accel_to_force_xy(double accel_xy)
  { return ORCA_MASS * accel_xy; }

  constexpr double accel_to_force_z(double accel_z)
  { return ORCA_MASS * accel_z; }

  constexpr double accel_to_torque_yaw(double accel_yaw)
  { return MOMENT_OF_INERTIA_YAW * accel_yaw; }

  // Acceleration => effort
  constexpr double accel_to_effort_xy(double accel_xy)
  { return force_to_effort_xy(accel_to_force_xy(accel_xy)); }

  constexpr double accel_to_effort_z(double accel_z)
  { return force_to_effort_z(accel_to_force_z(accel_z)); }

  constexpr double accel_to_effort_yaw(double accel_yaw)
  { return torque_to_effort_yaw(accel_to_torque_yaw(accel_yaw)); }

} // namespace orca_base

#endif // ORCA_BASE_MODEL_HPP
