#ifndef ORCA_BASE_SEGMENT_COMMON_HPP
#define ORCA_BASE_SEGMENT_COMMON_HPP

#include "orca_base/auv_context.hpp"
#include "orca_shared/geometry.hpp"
#include "rclcpp/duration.hpp"

namespace orca_base
{

  /**
   * Segments describe a trajectory from start to goal over time
   */
  class SegmentBase
  {
  protected:

    AUVContext cxt_;
    uint8_t type_;

  public:

    explicit SegmentBase(AUVContext cxt, uint8_t type);

    // Return a string suitable for logging
    virtual std::string to_str() = 0;

    // Advance the motion plan by dt seconds, return true to continue, false if we're done
    virtual bool advance(const rclcpp::Duration &d) = 0;

    __uint8_t type()
    { return type_; }

    std::string type_name();
  };

  /**
 * Trapezoidal velocity motion planner
 *
 * Phase 1: constant acceleration from p0 to p1
 * Phase 2: constant velocity from p1 to p2
 * Phase 3: constant deceleration from p2 to p3
 *
 * All dimensions are moving through the phases at the same time
 * If the distances are short, the constant velocity phase might be skipped
 * Assumes start velocity (v0) is {0, 0, 0, 0}
 *
 * @param cxt AUV context object with various parameters
 * @param p0 In: start of motion
 * @param p1 Out: pose and time at end of phase 1
 * @param p2 Out: pose and time at end of phase 2
 * @param p3 In: pose at end of phase 3, out: time at end of phase 3
 * @param a0 Out: acceleration, apply from p0.t to p1.t, and decelerate from p2.t to p3.t
 * @param v1 Out: peak velocity runs from p1.t to p2.t
 */
  void plan_sync(const AUVContext &cxt, const orca::PoseStamped &p0,
                 orca::PoseStamped &p1, orca::PoseStamped &p2, orca::PoseStamped &p3,
                 orca::Acceleration &a0, orca::Twist &v1);

} // namespace orca_base

#endif // ORCA_BASE_SEGMENT_COMMON_HPP