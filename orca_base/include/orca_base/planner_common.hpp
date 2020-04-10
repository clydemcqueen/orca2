#ifndef ORCA_BASE_PLANNER_COMMON_HPP
#define ORCA_BASE_PLANNER_COMMON_HPP

#include <orca_shared/mw/mw.hpp>
#include "orca_shared/mw/efforts.hpp"
#include "orca_shared/mw/mission_state.hpp"

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

    virtual bool advance(const rclcpp::Duration &d, const mw::FiducialPoseStamped &estimate, mw::Efforts &efforts,
                         mw::MissionState &status) = 0;
  };

} // namespace orca_base

#endif //ORCA_BASE_PLANNER_COMMON_HPP
