#ifndef ORCA_BASE_MOVE_TO_MARKER_PLANNER_HPP
#define ORCA_BASE_MOVE_TO_MARKER_PLANNER_HPP

#include "rclcpp/logger.hpp"

#include "orca_base/base_context.hpp"
#include "orca_base/controller.hpp"
#include "orca_base/planner_common.hpp"
#include "orca_base/segment.hpp"

namespace orca_base
{

  //=====================================================================================
  // MoveToMarkerPlanner -- a recovery strategy
  //=====================================================================================

  class MoveToMarkerPlanner
  {
    rclcpp::Logger logger_;
    const BaseContext &cxt_;

    int marker_id_;                                                   // Target
    std::vector<std::shared_ptr<ObservationSegmentBase>> segments_;   // Motion segments
    std::shared_ptr<ObservationController> controller_;               // Motion controller

  public:

    MoveToMarkerPlanner(const rclcpp::Logger &logger, const BaseContext &cxt, const ObservationStamped &start,
                        PlannerStatus &status);

    bool advance(const rclcpp::Duration &d, FPStamped &plan, const FPStamped &estimate,
                 orca::Efforts &efforts, PlannerStatus &status);
  };

} // namespace orca_base

#endif //ORCA_BASE_MOVE_TO_MARKER_PLANNER_HPP
