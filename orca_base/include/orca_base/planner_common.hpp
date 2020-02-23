#ifndef ORCA_BASE_PLANNER_COMMON_HPP
#define ORCA_BASE_PLANNER_COMMON_HPP

#include "orca_base/fp.hpp"

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

  struct PlannerStatus
  {
    int targets_total{};          // Number of targets in global plan
    int target_idx{};             // Current target index
    int target_marker_id{};       // Current target marker id

    uint8_t planner{};            // Local planner running, see Control.msg for values

    int segments_total{};         // Number of segments in local plan
    int segment_idx{};            // Current segment index
    std::string segment_info;     // Current segment info string

    FPStamped pose;               // Planned pose
    orca::Twist twist;            // Planned twist
  };

  //=====================================================================================
  // Target
  //=====================================================================================

  struct Target
  {
    int marker_id;
    FP fp; // Hmmm... I think we can get by with just an orca::Pose

    Target() : marker_id{NOT_A_MARKER}
    {}

    Target(int _marker_id, FP _fp) : marker_id{_marker_id}, fp{std::move(_fp)}
    {}
  };

  std::ostream &operator<<(std::ostream &os, Target const &target);

} // namespace orca_base

#endif //ORCA_BASE_PLANNER_COMMON_HPP
