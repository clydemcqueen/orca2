#ifndef ORCA_BASE_PLANNER_COMMON_HPP
#define ORCA_BASE_PLANNER_COMMON_HPP

#include "orca_shared/fp.hpp"

namespace orca_base
{

  //=====================================================================================
  // AdvanceRC
  //=====================================================================================

  struct AdvanceRC
  {
    static constexpr int CONTINUE = 0;
    static constexpr int SUCCESS = 1;
    static constexpr int FAILURE = 2;
  };

  //=====================================================================================
  // PlannerStatus
  //=====================================================================================

  // Capture [most of] the plan status in Control.msg
  struct PlannerStatus
  {
    // Set by AUVNode:
    // mode
    // global_plan_idx

    int targets_total{};          // Number of targets in global plan
    int target_idx{};             // Current target index
    int target_marker_id{};       // Current target marker id

    uint8_t planner{};            // Local planner running, see Control.msg for values
    int local_plan_idx{};         // Number of attempts at a local or recovery plan

    int segments_total{};         // Number of segments in local plan
    int segment_idx{};            // Current segment index
    std::string segment_info;     // Current segment info string
    uint8_t segment_type{};       // Segment type, see Control.msg for values

    orca::FPStamped pose;         // Planned pose
    orca::Twist twist;            // Planned twist

    // New targets, called by GlobalPlanner()
    void first_target(int num_targets, int first_marker_id);

    // Next target
    void next_target(int);

    // New segments, called by PosePlanner() and MoveToMarkerPlanner()
    void first_segment(uint8_t _planner, int _segments_total, std::string _segment_info, uint8_t _segment_type);

    // Next segment
    void next_segment(std::string _segment_info, uint8_t _segment_type);
  };

  //=====================================================================================
  // Target
  //=====================================================================================

  struct Target
  {
    int marker_id;
    orca::FP fp; // Hmmm... I think we can get by with just an orca::Pose

    Target() : marker_id{orca::NOT_A_MARKER}
    {}

    Target(int _marker_id, orca::FP _fp) : marker_id{_marker_id}, fp{std::move(_fp)}
    {}
  };

  std::ostream &operator<<(std::ostream &os, Target const &target);

  //=====================================================================================
  // LocalPlannerType
  //=====================================================================================

  struct LocalPlannerType
  {
    static constexpr int POSE_PLANNER = 0;
    static constexpr int MTM_PLANNER = 1;
    // Future: 360 planner
  };

  //=====================================================================================
  // LocalPlanner -- abstract base class
  //=====================================================================================

  class LocalPlanner
  {
    int type_;

  public:

    explicit LocalPlanner(int type) : type_{type}
    {}

    bool is_pose_planner()
    { return type_ == LocalPlannerType::POSE_PLANNER; }

    bool is_mtm_planner()
    { return type_ == LocalPlannerType::MTM_PLANNER; }

    virtual bool advance(const rclcpp::Duration &d, const orca::FPStamped &estimate, orca::Efforts &efforts,
                         PlannerStatus &status) = 0;
  };

} // namespace orca_base

#endif //ORCA_BASE_PLANNER_COMMON_HPP
