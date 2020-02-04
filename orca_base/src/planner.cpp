#include "orca_base/planner.hpp"

#include <random>
#include <utility>

using namespace orca;

namespace orca_base
{

  // Replan if the estimate and the plan disagree
  constexpr double MAX_POSE_XY_ERR = 0.6;
  constexpr double MAX_GOOD_OBS_DISTANCE = 10;
  constexpr double MAX_OBS_YAW_ERR = 0.2;

  //=====================================================================================
  // Utilities
  //=====================================================================================

#if 0
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
#endif

  std::ostream &operator<<(std::ostream &os, Target const &target)
  {
    os << "{marker: " << target.marker_id << ", pose: " << target.fp.pose.pose << "}";
  }

  //=====================================================================================
  // LocalPlanner -- build a local plan to a target
  //=====================================================================================

  LocalPlanner::LocalPlanner(const rclcpp::Logger &logger, const BaseContext &cxt, const orca::FP &start,
                             Target target, Map map, bool keep_station) :
    logger_{logger}, cxt_{cxt}, target_{std::move(target)}, map_{std::move(map)}, keep_station_{keep_station},
    segment_idx_{0}, controller_{std::make_shared<PoseController>(cxt_)}
  {
    RCLCPP_INFO_STREAM(logger_, "local plan to: " << target);

    // Generate a series of waypoints to minimize dead reckoning
    std::vector<Pose> waypoints;
    if (map_.get_waypoints(start.pose.pose, target_.fp.pose.pose, waypoints)) {
      RCLCPP_INFO(logger_, "... through %d waypoints", waypoints.size() - 1);
    } else {
      waypoints.push_back(target_.fp.pose.pose);
    }

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
    add_rotate_segment(plan, target_.fp.pose.pose.yaw);

    // Keep station at the last target
    if (keep_station_) {
      add_keep_station_segment(plan, 1e6);
    }

    // Create a path to this target for diagnostics
    if (!segments_.empty()) {
      local_path_.header.frame_id = cxt_.map_frame_;
      local_path_.poses.clear();

      geometry_msgs::msg::PoseStamped pose_msg;
      for (auto &i : segments_) {
        local_path_.header.frame_id = cxt_.map_frame_;
        i->plan().pose.pose.to_msg(pose_msg.pose);
        local_path_.poses.push_back(pose_msg);
      }

      // Add last goal pose
      segments_.back()->goal().pose.pose.to_msg(pose_msg.pose);
      local_path_.poses.push_back(pose_msg);
    }

    assert(!segments_.empty());
    RCLCPP_INFO(logger_, "segment 1 of %d", segments_.size());
    segments_[0]->log_info();
  }

  void LocalPlanner::add_keep_station_segment(FP &plan, double seconds)
  {
    segments_.push_back(std::make_shared<Pause>(logger_, cxt_, plan, seconds));
  }

  void LocalPlanner::add_vertical_segment(FP &plan, double z)
  {
    segments_.push_back(PoseSegment::make_vertical(logger_, cxt_, plan, z));
  }

  void LocalPlanner::add_rotate_segment(FP &plan, double yaw)
  {
    segments_.push_back(PoseSegment::make_rotate(logger_, cxt_, plan, yaw));
  }

  void LocalPlanner::add_line_segment(FP &plan, double x, double y)
  {
    segments_.push_back(PoseSegment::make_line(logger_, cxt_, plan, x, y));
  }

  bool LocalPlanner::advance(double dt, FP &plan, const FP &estimate, orca::Efforts &efforts,
                             const std::function<void(double completed, double total)> &send_feedback)
  {
    // Update the plan
    if (segments_[segment_idx_]->advance(dt)) {
      // Continue in this segment
    } else if (++segment_idx_ < segments_.size()) {
      // Move to the next segment
      RCLCPP_INFO(logger_, "segment %d of %d", segment_idx_ + 1, segments_.size());
      segments_[segment_idx_]->log_info();
    } else {
      // Local plan is complete
      return false;
    }

    // Share plan with caller (useful for diagnostics)
    plan = segments_[segment_idx_]->plan();

    // Run PID controller and calculate efforts
    controller_->calc(dt, plan, estimate, segments_[segment_idx_]->ff(), efforts);

    return true;
  }

  //=====================================================================================
  // MoveToMarkerPlanner -- a recovery strategy
  //=====================================================================================

  MoveToMarkerPlanner::MoveToMarkerPlanner(const rclcpp::Logger &logger, const BaseContext &cxt,
                                           const Observation &start) :
    logger_{logger}, cxt_{cxt}, marker_id_{start.id}, segment_idx_{0},
    controller_{std::make_shared<MoveToMarkerController>(cxt_)}
  {
    Observation plan, goal;
    goal = start;

    // Rotate to face the marker
    plan = goal;
    goal.yaw = 0;
    segments_.push_back(std::make_shared<RotateToMarkerSegment>(logger_, cxt_, plan, goal));

    // Move toward the marker
    plan = goal;
    goal.distance = 1;
    segments_.push_back(std::make_shared<MoveToMarkerSegment>(logger_, cxt_, plan, goal));

    // Start
    segments_[segment_idx_]->log_info();
  }

  bool MoveToMarkerPlanner::advance(double dt, FP &plan, const FP &estimate, orca::Efforts &efforts,
                                    const std::function<void(double completed, double total)> &send_feedback)
  {
    // Advance the plan
    if (segments_[segment_idx_]->advance(dt)) {
      // Continue in this segment
    } else if (++segment_idx_ < segments_.size()) {
      // Move to next segment
      segments_[segment_idx_]->log_info();
    } else {
      // Recovery action is complete
      return false;
    }

    // Share plan with caller (useful for diagnostics)
    plan = {};
    plan.observations.push_back(segments_[segment_idx_]->plan());

    // Run the PID controller(s) and calculate efforts
    // If marker was not observed, estimate.obs.id == NOT_A_MARKER, and calc() will ignore PID outputs
    // TODO get plan_z from map
    orca::Observation estimate_obs;
    estimate.get_obs(marker_id_, estimate_obs);
    controller_->calc(dt, segments_[segment_idx_]->plan(), -0.5, estimate_obs, estimate.pose.pose.z,
                      segments_[segment_idx_]->ff(), efforts);

    return true;
  }

  //=====================================================================================
  // MissionPlanner
  //=====================================================================================

  MissionPlanner::MissionPlanner(const rclcpp::Logger &logger, const BaseContext &cxt, Map map,
                                 orca_description::Parser parser,
                                 const image_geometry::PinholeCameraModel &fcam_model) :
    logger_{logger}, cxt_{cxt}, map_{std::move(map)}, parser_{std::move(parser)}, fcam_model_{fcam_model},
    keep_station_{false}, target_idx_{0} //, recovery_{false}
  {}

  void MissionPlanner::predict_observations(orca::FP &plan) const
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

  void MissionPlanner::start_local_plan(const orca::FP &start)
  {
    recovery_planner_ = nullptr;

    local_planner_ = std::make_shared<LocalPlanner>(logger_, cxt_, start, targets_[target_idx_], map_,
                                                    target_idx_ == targets_.size() - 1 ? keep_station_ : false);
  }

  void MissionPlanner::start_recovery_plan(const orca::Observation &start)
  {
    local_planner_ = nullptr;

    recovery_planner_ = std::make_shared<MoveToMarkerPlanner>(logger_, cxt_, start);
  }

  int MissionPlanner::advance(double dt, FP &plan, const FP &estimate, orca::Efforts &efforts,
                              const std::function<void(double completed, double total)> &send_feedback)
  {
    // Bootstrap
    if (!local_planner_ && !recovery_planner_) {
      if (estimate.good_pose()) {
        RCLCPP_INFO(logger_, "start local plan from good pose");
        start_local_plan(estimate);
      } else {
        orca::Observation closest_observation;
        if (estimate.closest_obs(closest_observation) < MAX_GOOD_OBS_DISTANCE) {
          RCLCPP_INFO(logger_, "start recovery plan from good observation");
          start_recovery_plan(closest_observation);
        } else {
          RCLCPP_INFO(logger_, "no plan and no good marker observations");
          return AdvanceRC::FAILURE;
        }
      }
    }

    if (local_planner_) {

      // Advance the local plan
      if (local_planner_->advance(dt, plan, estimate, efforts, send_feedback)) {

        // Predict observations from the planned pose
        predict_observations(plan);

        // Is there a good pose?
        if (estimate.good_pose()) {

          // Good pose, make sure the plan & estimate are reasonably close
          if (estimate.distance_xy(plan) > MAX_POSE_XY_ERR) {
            RCLCPP_INFO(logger_, "large pose error, re-plan");
            start_local_plan(estimate);
            return AdvanceRC::CONTINUE;
          }

        } else {

          // Dead reckoning: no marker, or marker is too far away to calculate a good pose

          // It's possible to detect a marker even if it's too far away to get a good pose
          // We can compare the planned and estimated marker observations

          // First check: if we expect to see the target marker, and we do see the target marker,
          // and the yaw error is high, we can execute a move-to-marker strategy
          // This should keep the marker visible until we're close enough to get a good pose
          Observation plan_target_obs, estimate_target_obs;

          if (plan.get_obs(targets_[target_idx_].marker_id, plan_target_obs) &&
              estimate.get_obs(targets_[target_idx_].marker_id, estimate_target_obs)) {

            double err_distance = std::abs(plan_target_obs.distance - estimate_target_obs.distance);
            double err_yaw = std::abs(plan_target_obs.yaw - estimate_target_obs.yaw);

            if (plan_target_obs.distance < MAX_GOOD_OBS_DISTANCE &&
                estimate_target_obs.distance < MAX_GOOD_OBS_DISTANCE &&
                err_yaw > MAX_OBS_YAW_ERR) {
              RCLCPP_INFO(logger_, "poor pose, target marker in view but yaw error is high, recover");
              start_recovery_plan(estimate_target_obs);
              return AdvanceRC::CONTINUE;
            }
          }
        }

      } else {

        // Local plan is complete
        // Move to next target
        if (++target_idx_ < targets_.size()) {
          RCLCPP_INFO_STREAM(logger_, "next target: " << targets_[target_idx_]);
          send_feedback(target_idx_, targets_.size());

          if (estimate.good_pose()) {
            // Known good pose, plan immediately
            start_local_plan(estimate);
          } else {
            // Hmmm... maybe we'll get a good pose next call to advance()
            local_planner_ = nullptr;
          }
        } else {
          // Mission complete!
          return AdvanceRC::SUCCESS;
        }
      }

    } else {

      // In recovery. Success?
      if (estimate.good_pose()) {
        RCLCPP_INFO(logger_, "recovery succeeded");

        // Known good pose, plan this cycle
        start_local_plan(estimate);
      } else {
        // Advance the recovery plan
        if (!recovery_planner_->advance(dt, plan, estimate, efforts, send_feedback)) {
          RCLCPP_INFO(logger_, "recovery complete, no good pose, giving up");
          return AdvanceRC::FAILURE;
        }
      }
    }

    // If we got this far, continue the mission
    return AdvanceRC::CONTINUE;
  }

  void MissionPlanner::finish_global_plan()
  {
    RCLCPP_INFO_STREAM(logger_, targets_.size() << " targets:");
    for (auto &i : targets_) {
      RCLCPP_INFO_STREAM(logger_, i);
    }

    // Write global_path_
    global_path_.header.frame_id = cxt_.map_frame_;
    global_path_.poses.clear();
    geometry_msgs::msg::PoseStamped pose_msg;
    for (auto &i : targets_) {
      global_path_.header.frame_id = cxt_.map_frame_;
      i.fp.pose.pose.to_msg(pose_msg.pose);
      global_path_.poses.push_back(pose_msg);
    }
  }

  void MissionPlanner::plan_target(const orca::FP &fp, bool keep_station)
  {
    targets_.clear();
    target_idx_ = 0;

    keep_station_ = keep_station;
    targets_.emplace_back(orca::NOT_A_MARKER, fp);

    finish_global_plan();
  }

  void MissionPlanner::plan_floor_markers(bool random)
  {
    targets_.clear();
    target_idx_ = 0;

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

    finish_global_plan();
  }

  void MissionPlanner::plan_wall_markers(bool random)
  {
    targets_.clear();
    target_idx_ = 0;

    for (const auto &marker : map_.markers()) {
      Target target;
      // Target == marker
      target.marker_id = marker.id;
      target.fp.pose.pose.from_msg(marker.marker_f_map);

      // Target is in front of the marker
      target.fp.pose.pose.x += sin(target.fp.pose.pose.yaw) * cxt_.auv_xy_distance_;
      target.fp.pose.pose.y -= cos(target.fp.pose.pose.yaw) * cxt_.auv_xy_distance_;

      // Face the marker to get a good pose
      target.fp.pose.pose.yaw = norm_angle(target.fp.pose.pose.yaw + M_PI_2);

      targets_.push_back(target);
    }

    if (random) {
      // Shuffle targets
      std::random_device rd;
      std::mt19937 g(rd());
      std::shuffle(targets_.begin(), targets_.end(), g);
    }

    finish_global_plan();
  }

} // namespace orca_base
