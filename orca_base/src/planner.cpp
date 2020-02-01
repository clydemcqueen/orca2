#include "orca_base/planner.hpp"

#include <random>

using namespace orca;

namespace orca_base
{

  // Replan if the estimate and the plan disagree
  constexpr double MAX_POSE_XY_ERR = 0.6;
  constexpr double MAX_OBS_YAW_ERR = M_PI_4;
  constexpr double MAX_OBS_DISTANCE_ERR = 1;

  //=====================================================================================
  // Utilities
  //=====================================================================================

  std::vector<std::pair<orca::Observation, orca::Observation>>
  find_common(std::vector<orca::Observation> o1, std::vector<orca::Observation> o2)
  {
    std::vector<std::pair<orca::Observation, orca::Observation>> r;

    auto compare = [](const Observation &a, const Observation &b)
    {
      return a.id > b.id;
    };

    std::sort(o1.begin(), o1.end(), compare);
    std::sort(o2.begin(), o2.end(), compare);

    auto o1i = o1.begin();
    auto o2i = o2.begin();
    while (o1i != o1.end() && o2i != o2.end()) {
      if (o1i->id == o2i->id) {
        r.emplace_back(*o1i, *o2i);
        o1i++;
        o2i++;
      } else if (o1i->id > o2i->id) {
        o2i++;
      } else {
        o1i++;
      }
    }

    return r;
  }

  //=====================================================================================
  // Planner
  //=====================================================================================

  Planner::Planner(const rclcpp::Logger &logger, const BaseContext &cxt, Map map, orca_description::Parser parser,
                   const image_geometry::PinholeCameraModel &fcam_model) :
    logger_{logger}, cxt_{cxt}, map_{std::move(map)}, parser_{std::move(parser)}, fcam_model_{fcam_model},
    keep_station_{false}, target_idx_{0}, segment_idx_{0}, recovery_{false}
  {
    controller_ = std::make_shared<PoseController>(cxt_);
    recovery_controller_ = std::make_shared<MoveToMarkerController>(cxt_);
  }

  void Planner::add_keep_station_segment(FP &plan, double seconds)
  {
    segments_.push_back(std::make_shared<Pause>(logger_, cxt_, plan, seconds));
  }

  void Planner::add_vertical_segment(FP &plan, double z)
  {
    FP goal = plan;
    goal.pose.pose.z = z;
    if (plan.pose.pose.distance_z(goal.pose.pose) > EPSILON_PLAN_XYZ) {
      segments_.push_back(std::make_shared<VerticalSegment>(logger_, cxt_, plan, goal));
    } else {
      RCLCPP_INFO(logger_, "skip vertical");
    }
    plan = goal;
  }

  void Planner::add_rotate_segment(FP &plan, double yaw)
  {
    FP goal = plan;
    goal.pose.pose.yaw = yaw;
    if (plan.pose.pose.distance_yaw(goal.pose.pose) > EPSILON_PLAN_YAW) {
      segments_.push_back(std::make_shared<RotateSegment>(logger_, cxt_, plan, goal));
    } else {
      RCLCPP_INFO(logger_, "skip rotate");
    }
    plan = goal;
  }

  void Planner::add_line_segment(FP &plan, double x, double y)
  {
    FP goal = plan;
    goal.pose.pose.x = x;
    goal.pose.pose.y = y;
    if (plan.pose.pose.distance_xy(goal.pose.pose) > EPSILON_PLAN_XYZ) {
      segments_.push_back(std::make_shared<LineSegment>(logger_, cxt_, plan, goal));
    } else {
      RCLCPP_INFO(logger_, "skip line");
    }
    plan = goal;
  }

  void Planner::plan_trajectory(const FP &start)
  {
    RCLCPP_INFO_STREAM(logger_, "local plan to " << targets_[target_idx_].fp.pose.pose);
    RCLCPP_INFO_STREAM(logger_, "marker id " << targets_[target_idx_].marker_id);

    // Generate a series of waypoints to minimize dead reckoning
    std::vector<Pose> waypoints;
    if (!map_.get_waypoints(start.pose.pose, targets_[target_idx_].fp.pose.pose, waypoints)) {
      RCLCPP_INFO(logger_, "feeling lucky");
      waypoints.push_back(targets_[target_idx_].fp.pose.pose);
    }

    // Plan trajectory through the waypoints
    plan_trajectory(waypoints, start);
  }

  void Planner::plan_trajectory(const std::vector<Pose> &waypoints, const FP &start)
  {
    RCLCPP_INFO(logger_, "local plan through %d waypoint(s):", waypoints.size() - 1);
    for (auto waypoint : waypoints) {
      RCLCPP_INFO_STREAM(logger_, waypoint);
    }

    // Clear segments
    segments_.clear();
    segment_idx_ = 0;

    // Start pose
    FP plan = start;

    // Travel to each waypoint, breaking down z, yaw and xy phases
    for (auto &waypoint : waypoints) {
      // Ascend/descend to target z
      add_vertical_segment(plan, waypoint.z);

      if (plan.pose.pose.distance_xy(waypoint.x, waypoint.y) > EPSILON_PLAN_XYZ) {
        // Point in the direction of travel
        add_rotate_segment(plan, atan2(waypoint.y - plan.pose.pose.y, waypoint.x - plan.pose.pose.x));

        // Travel
        add_line_segment(plan, waypoint.x, waypoint.y);
      } else {
        RCLCPP_INFO(logger_, "skip travel");
      }
    }

    // Always rotate to the target yaw
    add_rotate_segment(plan, targets_[target_idx_].fp.pose.pose.yaw);

    // Keep station at the last target
    if (keep_station_ && target_idx_ == targets_.size() - 1) {
      add_keep_station_segment(plan, 1e6);
    }

    // Create a path to this target for diagnostics
    if (!segments_.empty()) {
      planned_path_.header.frame_id = cxt_.map_frame_;
      planned_path_.poses.clear();

      geometry_msgs::msg::PoseStamped pose_msg;
      for (auto &i : segments_) {
        planned_path_.header.frame_id = cxt_.map_frame_;
        i->plan().pose.pose.to_msg(pose_msg.pose);
        planned_path_.poses.push_back(pose_msg);
      }

      // Add last goal pose
      segments_.back()->goal().pose.pose.to_msg(pose_msg.pose);
      planned_path_.poses.push_back(pose_msg);
    }

    assert(!segments_.empty());
    RCLCPP_INFO(logger_, "segment 1 of %d", segments_.size());
    segments_[0]->log_info();
  }

  void Planner::recover(const orca::Observation &start)
  {
    // Move to recovery mode
    segments_.clear();
    segment_idx_ = 0;
    recovery_ = true;

    orca::Observation plan = start;
    orca::Observation goal = plan;
    goal.distance = 1;
    goal.yaw = 0;

    recovery_segment_ = std::make_shared<MoveToMarkerSegment>(logger_, cxt_, plan, goal);

    RCLCPP_INFO_STREAM(logger_, "RECOVER! move to marker " << start.id << ", distance " << start.distance);
    recovery_segment_->log_info();
  }

  void Planner::predict_observations(orca::FP &plan) const
  {
    // Get t_fcam_map
    // This method works whether or not t_base_map is published

    geometry_msgs::msg::Pose base_f_map;
    plan.pose.pose.to_msg(base_f_map);

    tf2::Transform t_map_base;
    tf2::fromMsg(base_f_map, t_map_base);

    tf2::Transform t_base_map = t_map_base.inverse();

    tf2::Transform t_fcam_map;
    t_fcam_map = parser_.t_fcam_base * t_base_map;

    plan.observations.clear();

    int num_observations = 0;
    auto markers = map_.markers();
    for (const auto &marker : markers) {
      Observation obs;
      if (marker.predict_observation(fcam_model_, t_fcam_map, obs)) {
        ++num_observations;
        plan.observations.push_back(obs);
      }
    }

    // RCLCPP_INFO(logger_, "Predicted %d observation(s)", num_observations);
  }

  int Planner::advance_recovery(double dt, FP &plan, const FP &estimate, orca::Efforts &efforts,
                                const std::function<void(double completed, double total)> &send_feedback)
  {
    if (estimate.good_pose()) {
      RCLCPP_INFO(logger_, "recovery succeeded, replan to target");

      recovery_segment_ = nullptr;
      recovery_ = false;
      plan_trajectory(estimate); // Replan immediately, next estimate might not contains a good pose
      return AdvanceRC::CONTINUE;
    }

    if (!recovery_segment_->advance(dt)) {
      RCLCPP_ERROR(logger_, "recovery failed");
      return AdvanceRC::FAILURE;
    }

    // Look for the marker in the list of observations
    orca::Observation estimate_obs;
    if (estimate.get_obs(recovery_segment_->plan().id, estimate_obs)) {
      // We observed the marker, compute efforts to keep it in view
      auto recovery_plan_obs = recovery_segment_->plan();
      auto ff = recovery_segment_->ff();
      recovery_controller_->calc(dt, recovery_plan_obs, estimate_obs, ff, efforts);
    } else {
      RCLCPP_WARN(logger_, "didn't see the marker during recovery, will try again");
    }

    return AdvanceRC::CONTINUE;
  }

  int Planner::advance_plan(double dt, FP &plan, const FP &estimate, orca::Efforts &efforts,
                            const std::function<void(double completed, double total)> &send_feedback)
  {
    // Bootstrap
    if (segments_.empty()) {
      // Set targets before running the plan!
      assert(!targets_.empty());

      if (estimate.good_pose()) {
        // Generate a trajectory to the first target
        RCLCPP_INFO(logger_, "bootstrap local plan");
        plan_trajectory(estimate);
      } else {
        RCLCPP_ERROR(logger_, "unknown pose, can't bootstrap local plan");
        return AdvanceRC::FAILURE;
      }
    }

    // Advance the plan
    if (segments_[segment_idx_]->advance(dt)) {

      // Continue in this segment

    } else if (++segment_idx_ < segments_.size()) {

      // Move to the next segment
      RCLCPP_INFO(logger_, "segment %d of %d", segment_idx_ + 1, segments_.size());
      segments_[segment_idx_]->log_info();

    } else if (++target_idx_ < targets_.size()) {

      // Move to the next target
      RCLCPP_INFO(logger_, "target %d of %d", target_idx_ + 1, targets_.size());
      RCLCPP_INFO_STREAM(logger_, "marker id " << targets_[target_idx_].marker_id);
      send_feedback(target_idx_, targets_.size());

      if (estimate.good_pose()) {
        // Start from known location
        plan_trajectory(estimate);
        RCLCPP_INFO(logger_, "planning for next target from known pose");
      } else {
        // Plan a trajectory as if the AUV is at the previous target (it probably isn't)
        // Future: run through recovery actions to find a good pose
        RCLCPP_WARN(logger_, "didn't find target, planning for next target anyway");
        plan_trajectory(targets_[target_idx_ - 1].fp);
      }

    } else {
      // Mission complete!
      return AdvanceRC::SUCCESS;
    }

    // Update plan
    plan = segments_[segment_idx_]->plan();

    // Pose error?
    if (estimate.good_pose() && estimate.pose.pose.distance_xy(plan.pose.pose) > MAX_POSE_XY_ERR) {
      RCLCPP_WARN(logger_, "large pose error, replan to existing target");
      plan_trajectory(estimate);
      return AdvanceRC::CONTINUE;
    }

    // Predict observations from the planned pose
    predict_observations(plan);

    // Observation error?
    Observation plan_obs, estimate_obs;
    if (plan.get_obs(targets_[target_idx_].marker_id, plan_obs) &&
        estimate.get_obs(targets_[target_idx_].marker_id, estimate_obs)) {
      double err_distance = std::abs(plan_obs.distance - estimate_obs.distance);
      double err_yaw = std::abs(plan_obs.yaw - estimate_obs.yaw);
      if (plan_obs.distance < 10. && estimate_obs.distance < 10. && (err_yaw > 0.2 || err_distance > 1.)) {
        RCLCPP_WARN(logger_, "large observation error for target marker %d, attempt recovery",
                    targets_[target_idx_].marker_id);
        recover(estimate_obs);
        return AdvanceRC::CONTINUE;
      }
    }

    // Compute efforts to keep to the plan
    controller_->calc(dt, plan, estimate, segments_[segment_idx_]->ff(), efforts);
    return AdvanceRC::CONTINUE;
  }

  int Planner::advance(double dt, FP &plan, const FP &estimate, orca::Efforts &efforts,
                       const std::function<void(double completed, double total)> &send_feedback)
  {
    if (recovery_) {
      return advance_recovery(dt, plan, estimate, efforts, send_feedback);
    } else {
      return advance_plan(dt, plan, estimate, efforts, send_feedback);
    }
  }

  void Planner::create_target_path()
  {
    target_path_.header.frame_id = cxt_.map_frame_;
    target_path_.poses.clear();

    geometry_msgs::msg::PoseStamped pose_msg;
    for (auto &i : targets_) {
      target_path_.header.frame_id = cxt_.map_frame_;
      i.fp.pose.pose.to_msg(pose_msg.pose);
      target_path_.poses.push_back(pose_msg);
    }
  }

  void Planner::plan_target(const orca::FP &fp, bool keep_station)
  {
    keep_station_ = keep_station;
    targets_.emplace_back(orca::NOT_A_MARKER, fp);
    create_target_path();
  }

  void Planner::plan_floor_markers(bool random)
  {
    // Targets are directly above the markers
    for (const auto &marker : map_.markers()) {
      Target target;
      target.marker_id = marker.id;
      target.fp.pose.pose.from_msg(marker.marker_f_map);
      target.fp.pose.pose.z = cxt_.auv_z_target_;
      targets_.push_back(target);
    }

    if (random) {
      // Shuffle targets
      std::random_device rd;
      std::mt19937 g(rd());
      std::shuffle(targets_.begin(), targets_.end(), g);
    }

    create_target_path();
  }

  void Planner::plan_wall_markers(bool random)
  {
    for (const auto &marker : map_.markers()) {
      Target target;
      // Target == marker
      target.marker_id = marker.id;
      target.fp.pose.pose.from_msg(marker.marker_f_map);

      // Target is in front of the marker
      target.fp.pose.pose.x += sin(target.fp.pose.pose.yaw) * cxt_.auv_xy_distance_;
      target.fp.pose.pose.y -= cos(target.fp.pose.pose.yaw) * cxt_.auv_xy_distance_;

      // Face the marker to get a good pose
      target.fp.pose.pose.yaw = -target.fp.pose.pose.yaw;

      targets_.push_back(target);
    }

    if (random) {
      // Shuffle targets
      std::random_device rd;
      std::mt19937 g(rd());
      std::shuffle(targets_.begin(), targets_.end(), g);
    }

    create_target_path();
  }

} // namespace orca_base
