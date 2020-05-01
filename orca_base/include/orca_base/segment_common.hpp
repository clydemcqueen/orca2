#ifndef ORCA_BASE_SEGMENT_COMMON_HPP
#define ORCA_BASE_SEGMENT_COMMON_HPP

#include "orca_base/auv_context.hpp"
#include "rclcpp/duration.hpp"

namespace orca_base
{

  //=====================================================================================
  // Segments describe a trajectory from start to goal over time
  //=====================================================================================

  class SegmentBase
  {
  protected:

    AUVContext cxt_;
    uint8_t type_;

  public:

    SegmentBase(AUVContext cxt, uint8_t type);

    // Return a string suitable for logging
    virtual std::string to_str() = 0;

    // Advance the motion plan by dt seconds, return true to continue, false if we're done
    virtual bool advance(const rclcpp::Duration &d) = 0;

    __uint8_t type()
    { return type_; }

    std::string type_name() const;
  };

  //=====================================================================================
  // Trapezoidal velocity motion has 3 phases:
  //
  // Phase 1: constant acceleration from p0 to p1
  // Phase 2: constant velocity from p1 to p2
  // Phase 3: constant deceleration from p2 to p3
  //
  // If the distance is short, the constant velocity phase might be skipped
  // Assumes start velocity (v0) is {0, 0, 0, 0}
  //=====================================================================================

  struct FastPlan
  {
    double t_ramp;    // Time (seconds) for ramp up (phase 1) and ramp down (phase 3)
    double t_run;     // Time (seconds) for run (phase 2)

    /**
     * Find the fastest trapezoidal velocity plan, given max acceleration and velocity
     * Total time = 2 * t_ramp + t_run
     *
     * @param angle True if d is an angle
     * @param d The distance to cover
     * @param a_max Constraint: maximum acceleration
     * @param v_max Constraint: maximum velocity
     */
    FastPlan(bool angle, double d, double a_max, double v_max);
  };

  std::ostream &operator<<(std::ostream &os, FastPlan const &p);

  struct SyncPlan
  {
    double a;         // Acceleration for ramp
    double v;         // Velocity for run
    double d_ramp;    // Distance covered during ramp
    double d_run;     // Distance covered during run

    /**
     * Find a trapezoidal velocity plan with a given t_ramp and t_run
     *
     * @param angle True if d is an angle
     * @param d The distance to cover
     * @param t_ramp Ramp time
     * @param t_run Run time
     */
    SyncPlan(bool angle, double d, double t_ramp, double t_run);
  };

  std::ostream &operator<<(std::ostream &os, SyncPlan const &p);

} // namespace orca_base

#endif // ORCA_BASE_SEGMENT_COMMON_HPP