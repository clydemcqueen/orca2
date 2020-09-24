// Copyright (c) 2020, Clyde McQueen.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef ORCA_SHARED__MODEL_HPP_
#define ORCA_SHARED__MODEL_HPP_

#include <cmath>

#include "rclcpp/logger.hpp"
#include "ros2_shared/context_macros.hpp"

namespace orca
{

// World frame suffixes:
// _x
// _y
// _xy for a value in the x-y plane, e.g., a distance to a point

// Body frame suffixes:
// _f forward
// _s strafe
// _fs for a value in the forward-strafe plane, e.g., a distance to a point

#define MODEL_PARAMS \
  CXT_MACRO_MEMBER(mdl_mass, double, 9.75) \
  CXT_MACRO_MEMBER(mdl_volume, double, 0.01) \
  CXT_MACRO_MEMBER(mdl_fluid_density, double, 997) \
  /* kg/m^3, 997 for freshwater, 1029 for seawater  */ \
  CXT_MACRO_MEMBER(mdl_thrust_scale, double, 0.7) \
  /* Scale max thruster forces to give a better linear approximation  */ \
  CXT_MACRO_MEMBER(mdl_drag_coef_f, double, 0.8) \
  /* Forward drag, 1.0 is a box  */ \
  CXT_MACRO_MEMBER(mdl_drag_coef_s, double, 0.95) \
  /* Strafe drag  */ \
  CXT_MACRO_MEMBER(mdl_drag_coef_z, double, 0.95) \
  /* Vertical drag  */ \
  CXT_MACRO_MEMBER(mdl_drag_coef_tether, double, 1.1) \
  /* Tether drag, 1.2 for unfaired tether  */ \
  CXT_MACRO_MEMBER(mdl_drag_partial_const_yaw, double, 0.004) \
  /* Yaw drag, wild guess  */ \
  CXT_MACRO_MEMBER(mdl_thrust_dz_pwm, uint16_t, 35) \
  /* Thruster deadzone  */                 \
/* End of list */

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)

struct Model
{
  //=====================================================================================
  // Physics constants
  //=====================================================================================

  static constexpr double
    GRAVITY = 9.8;                    // Gazebo default is 9.8, 9.80665 is a bit more accurate

  //=====================================================================================
  // Sensor constants
  //=====================================================================================

  static constexpr double BARO_STDDEV = 201.7;      // Measured during ft3
  static constexpr double BARO_FREQ = 23.0;
  static constexpr double DEPTH_STDDEV = 0.02064;   // Measured during ft3
  static constexpr double CAMERA_FREQ = 30.0;

  //=====================================================================================
  // Vehicle constants, in the body frame (x forward, y left, z up)
  //=====================================================================================

  static constexpr double ROV_DIM_F = 0.457;       // Length
  static constexpr double ROV_DIM_S = 0.338;       // Width
  static constexpr double ROV_DIM_Z = 0.254;       // Height

  static constexpr double TETHER_DIAM = 0.008;

  static constexpr double
    ROV_AREA_BOW = ROV_DIM_S * ROV_DIM_Z;   // Area of front (bow) and rear (stern) sides
  static constexpr double ROV_AREA_TOP = ROV_DIM_F * ROV_DIM_Z;   // Area of top and bottom
  static constexpr double
    ROV_AREA_PORT = ROV_DIM_F * ROV_DIM_S;  // Area of left (port) and right (starboard) sides

  // From BlueRobotics specs, in Newtons
  static constexpr double T200_MAX_POS_FORCE = 50;
  static constexpr double T200_MAX_NEG_FORCE = 40;

  // bollard_force_xy_ could also be called bollard_force_fs_
  static constexpr double BOLLARD_FORCE_XY = 0.76 * 2 * (T200_MAX_POS_FORCE + T200_MAX_NEG_FORCE);

  // Up and down forces are different
  static constexpr double BOLLARD_FORCE_Z_UP = 2 * T200_MAX_POS_FORCE;
  static constexpr double BOLLARD_FORCE_Z_DOWN = 2 * T200_MAX_NEG_FORCE;

  // Estimate maximum yaw torque by looking at 4 thrusters (2 forward, 2 reverse),
  // each mounted ~tangent to a circle with radius = 18cm
  static constexpr double MAX_TORQUE_YAW = 0.18 * 2 * (T200_MAX_POS_FORCE + T200_MAX_NEG_FORCE);

  //=====================================================================================
  // Parameters
  //=====================================================================================

  MODEL_PARAMS

  //=====================================================================================
  // Dynamics
  //=====================================================================================

  // Assume a uniform distribution of mass in the vehicle box
  double moment_of_inertia_yaw_ = mdl_mass_ / 12.0 * (ROV_DIM_F * ROV_DIM_F + ROV_DIM_S * ROV_DIM_S);

  // Force / torque => acceleration
  double force_to_accel(double force) const {return force / mdl_mass_;}

  double torque_to_accel_yaw(double torque_yaw) const
  {
    return torque_yaw / moment_of_inertia_yaw_;
  }

  // Acceleration => force / torque
  double accel_to_force(double accel) const {return mdl_mass_ * accel;}

  double accel_to_torque_yaw(double accel_yaw) const {return moment_of_inertia_yaw_ * accel_yaw;}

  //=====================================================================================
  // Force / torque <=> effort
  // Effort is force / max_force, and ranges from -1.0 to 1.0
  //=====================================================================================

  double bollard_force_xy() const {return BOLLARD_FORCE_XY * mdl_thrust_scale_;}

  double bollard_force_z_up() const {return BOLLARD_FORCE_Z_UP * mdl_thrust_scale_;}

  double bollard_force_z_down() const {return BOLLARD_FORCE_Z_DOWN * mdl_thrust_scale_;}

  double max_torque_yaw() const {return MAX_TORQUE_YAW * mdl_thrust_scale_;}

  double force_to_effort_xy(double force_xy) const
  {
    return force_xy / bollard_force_xy();
  }

  double force_to_effort_z(double force_z) const
  {
    return force_z / (force_z > 0 ? bollard_force_z_up() : bollard_force_z_down());
  }

  double torque_to_effort_yaw(double torque_yaw) const
  {
    return torque_yaw / max_torque_yaw();
  }

  double effort_to_force_xy(double effort_xy) const
  {
    return effort_xy * bollard_force_xy();
  }

  double effort_to_force_z(double effort_z) const
  {
    return effort_z * (effort_z > 0 ? bollard_force_z_up() : bollard_force_z_down());
  }

  double effort_to_torque_yaw(double effort_yaw) const
  {
    return effort_yaw * max_torque_yaw();
  }

  //=====================================================================================
  // Acceleration <=> effort
  //=====================================================================================

  double accel_to_effort_xy(double accel_xy) const
  {
    return force_to_effort_xy(accel_to_force(accel_xy));
  }

  double accel_to_effort_z(double accel_z) const
  {
    return force_to_effort_z(accel_to_force(accel_z));
  }

  double accel_to_effort_yaw(double accel_yaw) const
  {
    return torque_to_effort_yaw(accel_to_torque_yaw(accel_yaw));
  }

  double effort_to_accel_xy(double effort_xy) const
  {
    return force_to_accel(effort_to_force_xy(effort_xy));
  }

  double effort_to_accel_z(double effort_z) const
  {
    return force_to_accel(effort_to_force_z(effort_z));
  }

  double effort_to_accel_yaw(double effort_yaw) const
  {
    return torque_to_accel_yaw(effort_to_torque_yaw(effort_yaw));
  }

  //=====================================================================================
  // Water pressure depends on fluid density
  //=====================================================================================

  double pressure_to_z(double atmospheric_pressure, double pressure) const
  {
    return -(pressure - atmospheric_pressure) / (mdl_fluid_density_ * GRAVITY);
  }

  double z_to_pressure(double atmospheric_pressure, double z) const
  {
    return mdl_fluid_density_ * GRAVITY * -z + atmospheric_pressure;
  }

  double atmospheric_pressure(double pressure, double z) const
  {
    return pressure - mdl_fluid_density_ * GRAVITY * -z;
  }

  // Mass displaced by the volume, in kg
  double displaced_mass() const {return mdl_volume_ * mdl_fluid_density_;}

  // Weight in water, in Newtons (kg * m/s^2)
  double weight_in_water() const {return GRAVITY * (mdl_mass_ - displaced_mass());}

  // Z acceleration required to hover, in m/s^2
  double hover_accel_z() const {return weight_in_water() / mdl_mass_;}

  //=====================================================================================
  // Drag in the body frame (x forward, y left, z up)
  // = 0.5 * density * area * velocity^2 * coefficient
  //
  // The ROV constants below capture all but velocity:
  //    drag constant = 0.5 * density * area * coefficient
  //
  // The tether constant below captures all but depth and velocity:
  //    drag constant = 0.5 * density * width * coefficient
  //=====================================================================================

  double drag_const_f() const {return 0.5 * mdl_fluid_density_ * ROV_AREA_BOW * mdl_drag_coef_f_;}

  double drag_const_s() const {return 0.5 * mdl_fluid_density_ * ROV_AREA_PORT * mdl_drag_coef_s_;}

  double drag_const_z() const {return 0.5 * mdl_fluid_density_ * ROV_AREA_TOP * mdl_drag_coef_z_;}

  double drag_const_yaw() const {return mdl_fluid_density_ * mdl_drag_partial_const_yaw_;}

  double tether_drag_const() const
  {
    return 0.5 * mdl_fluid_density_ * TETHER_DIAM * mdl_drag_coef_tether_;
  }

  //=====================================================================================
  // Drag force, and acceleration-due-to-drag, in the body frame (x forward, y left, z up)
  //=====================================================================================

  // Velocity => drag force / torque
  // Motion works in all 4 quadrants, note the use of abs()
  static double drag_force(double velo, double drag_constant)
  {
    return velo * std::abs(velo) * -drag_constant;
  }

  double drag_force_f(double velo_f) const {return drag_force(velo_f, drag_const_f());}

  double drag_force_s(double velo_s) const {return drag_force(velo_s, drag_const_s());}

  double drag_force_z(double velo_z) const {return drag_force(velo_z, drag_const_z());}

  double drag_torque_yaw(double velo_yaw) const
  {
    return velo_yaw * std::abs(velo_yaw) * -drag_const_yaw();
  }

  // Velocity => acceleration due to drag
  double drag_accel(double velo, double drag_constant) const
  {
    return force_to_accel(drag_force(velo, drag_constant));
  }

  double drag_accel_f(double velo_f) const {return drag_accel(velo_f, drag_const_f());}

  double drag_accel_s(double velo_s) const {return drag_accel(velo_s, drag_const_s());}

  double drag_accel_z(double velo_z) const {return drag_accel(velo_z, drag_const_z());}

  double drag_accel_yaw(double velo_yaw) const
  {
    return torque_to_accel_yaw(drag_torque_yaw(velo_yaw));
  }

  //=====================================================================================
  // Drag constants in the world frame: drag_const_x, drag_const_y
  //    yaw is the orientation of the body in the world frame
  //    motion_world is the direction of motion in the world frame
  //
  // Example: if yaw == direction_of_motion, the body is moving forward, therefore:
  //    drag_const_x = drag_const_f()
  //    drag_const_y = drag_const_s()
  //=====================================================================================

  void
  drag_const_world(double yaw, double motion_world, double & drag_const_x,
    double & drag_const_y) const;

  // Log some info... handy for debugging
  void log_info(const rclcpp::Logger & logger) const;
};

}  // namespace orca

#endif  // ORCA_SHARED__MODEL_HPP_
