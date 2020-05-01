#ifndef ORCA_BASE_POSE_SEGMENT_HPP
#define ORCA_BASE_POSE_SEGMENT_HPP

#include "orca_base/segment_common.hpp"
#include "orca_shared/mw/mw.hpp"

namespace orca_base
{

  //=====================================================================================
  // plan_pose_sync
  //=====================================================================================

  /**
   * Trapezoidal velocity motion planner for 4DoF poses
   * All DoF are moving through the phases at the same time
   *
   * @param cxt AUV context object with various parameters
   * @param p0 In: start of motion
   * @param p1 Out: pose and time at end of phase 1
   * @param p2 Out: pose and time at end of phase 2
   * @param p3 In: pose at end of phase 3, out: time at end of phase 3
   * @param a0 Out: acceleration, apply from p0.t to p1.t, and decelerate from p2.t to p3.t
   * @param v1 Out: peak velocity runs from p1.t to p2.t
   */
  void plan_pose_sync(const AUVContext &cxt, const mw::PoseStamped &p0,
                      mw::PoseStamped &p1, mw::PoseStamped &p2, mw::PoseStamped &p3,
                      mw::Acceleration &a0, mw::Twist &v1);

  //=====================================================================================
  // Pose segments plan motion based on poses
  //=====================================================================================

  class PoseSegmentBase : public SegmentBase
  {
  protected:

    mw::PoseStamped plan_;      // Planned pose, incremented with each call to advance()
    mw::Pose goal_;             // Goal pose

    mw::Twist twist_;           // Velocity in the world frame
    mw::Acceleration ff_;       // Acceleration in the world frame

  public:
    PoseSegmentBase(const AUVContext &cxt, uint8_t type, mw::PoseStamped start, mw::Pose goal);

    const mw::PoseStamped &plan() const
    { return plan_; }

    const mw::Pose &goal() const
    { return goal_; }

    const mw::Twist &twist() const
    { return twist_; }

    const mw::Acceleration &ff() const
    { return ff_; }
  };

  //=====================================================================================
  // Pause segments stay in one pose for a period of time
  //=====================================================================================

  class Pause : public PoseSegmentBase
  {
    rclcpp::Duration pause_duration_;     // Time remaining

  public:

    Pause(const AUVContext &cxt, const mw::PoseStamped &start, const rclcpp::Duration &pause_duration);

    std::string to_str() override;

    bool advance(const rclcpp::Duration &d) override;
  };

  //=====================================================================================
  // Trap2: implement a trapezoidal velocity motion segment
  //=====================================================================================

  class Trap2 : public PoseSegmentBase
  {
    mw::PoseStamped p0_;    // Start pose
    mw::PoseStamped p1_;
    mw::PoseStamped p2_;
    mw::PoseStamped p3_;    // End pose
    mw::Acceleration a0_;   // Acceleration at p0_.t
    mw::Twist v1_;          // Velocity at p1_.t

  public:

    Trap2(const AUVContext &cxt, uint8_t type, const mw::PoseStamped &start, const mw::Pose &goal);

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
    static std::shared_ptr<Trap2> make_vertical(const AUVContext &cxt, mw::PoseStamped &plan, double z);

    /**
     * Construct a segment that rotates
     * @param cxt AUV context
     * @param plan In/out: plan
     * @param yaw In: incremental angle
     * @return Trap2 segment
     */
    static std::shared_ptr<Trap2> make_rotate(const AUVContext &cxt, mw::PoseStamped &plan, double yaw);

    /**
     * Construct a segment that moves in a line
     * @param cxt AUV context
     * @param plan In/out: plan
     * @param x In: x distance
     * @param y In: y distance
     * @return Trap2 segment
     */
    static std::shared_ptr<Trap2> make_line(const AUVContext &cxt, mw::PoseStamped &plan, double x, double y);

    /**
     * Construct a segment that rotates and moves in x, y and z
     * @param cxt AUV context
     * @param plan In/out: plan
     * @param goal In: goal pose
     * @return Trap2 segment
     */
    static std::shared_ptr<Trap2> make_pose(const AUVContext &cxt, mw::PoseStamped &plan, const mw::Pose &goal);
  };

} // namespace orca_base

#endif // ORCA_BASE_POSE_SEGMENT_HPP