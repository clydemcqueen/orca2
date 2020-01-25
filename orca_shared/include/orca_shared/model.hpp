#ifndef ORCA_SHARED_MODEL_HPP
#define ORCA_SHARED_MODEL_HPP

#include <cmath>

namespace orca
{
  struct Model
  {
    //=====================================================================================
    // Physics constants
    //=====================================================================================

    static constexpr double GRAVITY = 9.8;                    // Gazebo default is 9.8, 9.80665 is a bit more accurate
    static constexpr double ATMOSPHERIC_PRESSURE = 101300;    // Air pressure at the surface

    //=====================================================================================
    // Sensor constants
    //=====================================================================================

    static constexpr double DEPTH_STDDEV = 0.01;

    //=====================================================================================
    // Vehicle constants, in the body frame (x forward, y left, z up)
    //=====================================================================================

    static constexpr double ROV_DIM_X = 0.457;       // Length
    static constexpr double ROV_DIM_Y = 0.338;       // Width
    static constexpr double ROV_DIM_Z = 0.254;       // Height

    static constexpr double TETHER_DIAM = 0.008;

    static constexpr double ROV_AREA_X = ROV_DIM_Y * ROV_DIM_Z;  // Area of front (bow) and rear (stern) sides
    static constexpr double ROV_AREA_Y = ROV_DIM_X * ROV_DIM_Z;  // Area of top and bottom
    static constexpr double ROV_AREA_Z = ROV_DIM_X * ROV_DIM_Y;  // Area of left (port) and right (starboard) sides

    static constexpr double MASS = 9.75;
    static constexpr double VOLUME = 0.01;

    // Assume a uniform distribution of mass in the vehicle box
    static constexpr double MOMENT_OF_INERTIA_YAW = MASS / 12.0 * (ROV_DIM_X * ROV_DIM_X + ROV_DIM_Y * ROV_DIM_Y);

    // From BlueRobotics specs, all forces are in Newtons
    static constexpr double BOLLARD_FORCE_XY = 137;
    static constexpr double BOLLARD_FORCE_Z = 88;
    static constexpr double T200_MAX_POS_FORCE = 50;
    static constexpr double T200_MAX_NEG_FORCE = 40;

    // Estimate maximum yaw torque by looking at 4 thrusters (2 forward, 2 reverse), each mounted ~tangent to a circle with radius = 18cm
    static constexpr double MAX_TORQUE_YAW = 0.18 * 2.0 * (T200_MAX_POS_FORCE + T200_MAX_NEG_FORCE);

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

    static constexpr double DRAG_COEFFICIENT_X = 0.8;         // Estimated
    static constexpr double DRAG_COEFFICIENT_Y = 0.95;        // Estimated
    static constexpr double DRAG_COEFFICIENT_Z = 0.95;        // Estimated
    static constexpr double TETHER_DRAG_COEFFICIENT = 1.1;    // Estimated

    //=====================================================================================
    // Dynamics
    //=====================================================================================

    // Force / torque => acceleration
    static constexpr double force_to_accel(double force)
    { return force / MASS; }

    static constexpr double torque_to_accel_yaw(double torque_yaw)
    { return torque_yaw / MOMENT_OF_INERTIA_YAW; }

    // Force / torque => effort
    static constexpr double force_to_effort_xy(double force_xy)
    { return force_xy / BOLLARD_FORCE_XY; }

    static constexpr double force_to_effort_z(double force_z)
    { return force_z / BOLLARD_FORCE_Z; }

    static constexpr double torque_to_effort_yaw(double torque_yaw)
    { return torque_yaw / MAX_TORQUE_YAW; }

    // Effort => force / torque
    static constexpr double effort_to_force_xy(double effort_xy)
    { return effort_xy * BOLLARD_FORCE_XY; }

    static constexpr double effort_to_torque_yaw(double effort_yaw)
    { return effort_yaw * MAX_TORQUE_YAW; }

    static constexpr double effort_to_force_z(double effort_z)
    { return effort_z * BOLLARD_FORCE_Z; }

    // Acceleration => force / torque
    static constexpr double accel_to_force_xy(double accel_xy)
    { return MASS * accel_xy; }

    static constexpr double accel_to_force_z(double accel_z)
    { return MASS * accel_z; }

    static constexpr double accel_to_torque_yaw(double accel_yaw)
    { return MOMENT_OF_INERTIA_YAW * accel_yaw; }

    // Acceleration => effort
    static constexpr double accel_to_effort_xy(double accel_xy)
    { return force_to_effort_xy(accel_to_force_xy(accel_xy)); }

    static constexpr double accel_to_effort_z(double accel_z)
    { return force_to_effort_z(accel_to_force_z(accel_z)); }

    static constexpr double accel_to_effort_yaw(double accel_yaw)
    { return torque_to_effort_yaw(accel_to_torque_yaw(accel_yaw)); }

    // Effort => acceleration
    static constexpr double effort_to_accel_xy(double effort_xy)
    { return force_to_accel(effort_to_force_xy(effort_xy)); }

    static constexpr double effort_to_accel_z(double effort_z)
    { return force_to_accel(effort_to_force_z(effort_z)); }

    static constexpr double effort_to_accel_yaw(double effort_yaw)
    { return torque_to_accel_yaw(effort_to_torque_yaw(effort_yaw)); }

    //=====================================================================================
    // Parameters, updated by validate_parameters
    //=====================================================================================

    // Fluid density, 997 for freshwater or 1029 for seawater
    double fluid_density_ = 997;

    //=====================================================================================
    // Values which depend on the parameters
    //=====================================================================================

    double pressure_to_z(double pressure) const
    { return -(pressure - ATMOSPHERIC_PRESSURE) / (fluid_density_ * GRAVITY); }

    double z_to_pressure(double z) const
    { return fluid_density_ * GRAVITY * -z + ATMOSPHERIC_PRESSURE; }

    double displaced_mass() const
    { return VOLUME * fluid_density_; }

    double weight_in_water() const
    { return GRAVITY * (MASS - displaced_mass()); }

    // Z acceleration required to hover
    double hover_accel_z() const
    { return weight_in_water() / MASS; }

    double linear_drag_x() const
    { return 0.5 * fluid_density_ * ROV_AREA_X * DRAG_COEFFICIENT_X; }

    double linear_drag_y() const
    { return 0.5 * fluid_density_ * ROV_AREA_Y * DRAG_COEFFICIENT_Y; }

    double linear_drag_z() const
    { return 0.5 * fluid_density_ * ROV_AREA_Z * DRAG_COEFFICIENT_Z; }

    // Estimate angular drag
    double angular_drag_yaw() const
    { return 0.02 * (linear_drag_x() + linear_drag_y()); }

    double tether_drag() const
    { return 0.5 * fluid_density_ * TETHER_DIAM * TETHER_DRAG_COEFFICIENT; }

    // Velocity => drag force / torque
    double drag_force_x(double velo_x) const
    { return velo_x * std::abs(velo_x) * -linear_drag_x(); }

    double drag_force_y(double velo_y) const
    { return velo_y * std::abs(velo_y) * -linear_drag_y(); }

    double drag_force_z(double velo_z) const
    { return velo_z * std::abs(velo_z) * -linear_drag_z(); }

    double drag_torque_yaw(double velo_yaw) const
    { return velo_yaw * std::abs(velo_yaw) * -angular_drag_yaw(); }

    // Velocity => acceleration due to drag
    double drag_accel_x(double velo_x) const
    { return force_to_accel(drag_force_x(velo_x)); }

    double drag_accel_y(double velo_y) const
    { return force_to_accel(drag_force_y(velo_y)); }

    double drag_accel_z(double velo_z) const
    { return force_to_accel(drag_force_z(velo_z)); }

    double drag_accel_yaw(double velo_yaw) const
    { return torque_to_accel_yaw(drag_torque_yaw(velo_yaw)); }

  };

} // namespace orca

#endif //ORCA_SHARED_MODEL_HPP
