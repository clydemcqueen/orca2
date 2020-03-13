#ifndef ORCA_BASE_POSE_SEGMENT_HPP
#define ORCA_BASE_POSE_SEGMENT_HPP

#include "orca_base/fp.hpp"
#include "orca_base/segment_common.hpp"

namespace orca_base
{

  //=====================================================================================
  // Pose segments plan motion based on poses
  //=====================================================================================

  class PoseSegmentBase : public SegmentBase
  {
  protected:

    FPStamped plan_;        // Planned pose, incremented with each call to advance()
    FP goal_;               // Goal pose

    orca::Twist twist_;     // Velocity in the world frame
    orca::Acceleration ff_; // Acceleration in the world frame

  public:
    PoseSegmentBase(const AUVContext &cxt, uint8_t type, FPStamped start, FP goal);

    const FPStamped &plan() const
    { return plan_; }

    const FP &goal() const
    { return goal_; }

    const orca::Twist &twist() const
    { return twist_; }

    const orca::Acceleration &ff() const
    { return ff_; }
  };

  //=====================================================================================
  // Pause segments stay in one pose for a period of time
  //=====================================================================================

  class Pause : public PoseSegmentBase
  {
    rclcpp::Duration pause_duration_;     // Time remaining

  public:

    Pause(const AUVContext &cxt, const FPStamped &start, const rclcpp::Duration &pause_duration);

    std::string to_str() override;

    bool advance(const rclcpp::Duration &d) override;
  };

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

#endif // ORCA_BASE_POSE_SEGMENT_HPP