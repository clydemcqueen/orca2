#include "orca_base/segment.hpp"

namespace orca_base
{

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

  //=====================================================================================
  // Trap2: implement a trapezoidal velocity motion segment
  //=====================================================================================

  class Trap2 : public PoseSegmentBase
  {
    orca::PoseStamped p0_;    // Start pose
    orca::PoseStamped p1_;
    orca::PoseStamped p2_;
    orca::PoseStamped p3_;    // End pose
    orca::Acceleration a0_;   // Acceleration at p0_.t
    orca::Twist v1_;          // Velocity at p1_.t

  public:

    Trap2(const AUVContext &cxt, uint8_t type, const FPStamped &start, const FP &goal);

    /**
     * Duration of motion
     * @return duration of motion
     */
    rclcpp::Duration duration() const;

    std::string to_str() override;

    /**
     * Advance plan by d TODO pass in t, not d
     * @param d time increment
     * @return true if plan was advanced, false if plan is complete
     */
    bool advance(const rclcpp::Duration &d) override;

    /**
     * Construct a segment that moves up or down
     * @param cxt AUV context
     * @param plan In/out: plan
     * @param z In: vertical distance to travel
     * @return Trap2 segment
     */
    static std::shared_ptr<Trap2>
    make_vertical(const AUVContext &cxt, FPStamped &plan, double z);

    /**
     * Construct a segment that rotates
     * @param cxt AUV context
     * @param plan In/out: plan
     * @param yaw In: incremental angle
     * @return Trap2 segment
     */
    static std::shared_ptr<Trap2>
    make_rotate(const AUVContext &cxt, FPStamped &plan, double yaw);

    /**
     * Construct a segment that moves in a line
     * @param cxt AUV context
     * @param plan In/out: plan
     * @param x In: x distance
     * @param y In: y distance
     * @return Trap2 segment
     */
    static std::shared_ptr<Trap2>
    make_line(const AUVContext &cxt, FPStamped &plan, double x, double y);

    /**
     * Construct a segment that rotates and moves in x, y and z
     * @param cxt AUV context
     * @param plan In/out: plan
     * @param goal In: goal pose
     * @return Trap2 segment
     */
    static std::shared_ptr<Trap2>
    make_pose(const AUVContext &cxt, FPStamped &plan, const FP &goal);
  };

} // namespace orca_base