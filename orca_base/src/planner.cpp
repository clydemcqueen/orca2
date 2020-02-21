#include "orca_base/planner.hpp"

#include <random>
#include <utility>
#include <iomanip>

#include "orca_shared/util.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace orca;

namespace orca_base
{

  //=====================================================================================
  // Utilities
  //=====================================================================================

#if 0
  std::vector<std::pair<Observation, Observation>>
  find_common(std::vector<Observation> o1, std::vector<Observation> o2)
  {
    std::vector<std::pair<Observation, Observation>> r;

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
    os << std::fixed << std::setprecision(2)
       << "{marker: " << target.marker_id << ", pose: " << target.fp.pose.pose << "}";
  }

  //=====================================================================================
  // LocalPlanner -- build a local plan to a target
  //=====================================================================================

  LocalPlanner::LocalPlanner(const rclcpp::Logger &logger, const BaseContext &cxt, const FPStamped &start,
                             Target target, Map map, bool keep_station) :
    logger_{logger}, cxt_{cxt}, target_{std::move(target)}, map_{std::move(map)}, keep_station_{keep_station},
    segment_idx_{0}, controller_{std::make_shared<PoseController>(cxt_)}
  {
    // Future: build a plan that will keep markers in view at all times
    // For now, consider 2 cases:
    // 1. short plan: use a single segment to move from start to target
    // 2. long plan: find a series of waypoints (if possible), and always face the direction of motion

    // Start pose
    FPStamped plan = start;

    if (target.fp.distance_xy(start.fp) < cxt_.planner_max_short_plan_xy_) {
      RCLCPP_INFO_STREAM(logger_, "short plan: " << target);

      add_pose_segment(plan, target.fp);

    } else {
      RCLCPP_INFO_STREAM(logger_, "long plan: " << target);

      // Generate a series of waypoints to minimize dead reckoning
      std::vector<Pose> waypoints;

      if (cxt_.planner_look_for_waypoints_ && map_.get_waypoints(start.fp.pose.pose, target_.fp.pose.pose, waypoints)) {
        RCLCPP_INFO(logger_, "... through %d waypoints", waypoints.size() - 1);
      } else {
        waypoints.push_back(target_.fp.pose.pose);
      }

      // Travel to each waypoint, breaking down z, yaw and xy phases
      for (auto &waypoint : waypoints) {
        // Ascend/descend to target z
        add_vertical_segment(plan, waypoint.z);

        if (plan.fp.pose.pose.distance_xy(waypoint.x, waypoint.y) > cxt_.planner_epsilon_xyz_) {
          // Point in the direction of travel
          add_rotate_segment(plan, atan2(waypoint.y - plan.fp.pose.pose.y, waypoint.x - plan.fp.pose.pose.x));

          // Travel
          add_line_segment(plan, waypoint.x, waypoint.y);
        } else {
          RCLCPP_INFO(logger_, "skip travel");
        }
      }

      // Always rotate to the target yaw
      add_rotate_segment(plan, target_.fp.pose.pose.yaw);
    }

    // Pause
    add_keep_station_segment(plan, 5);

    // Create a path message to this target for diagnostics
    local_path_.header.frame_id = cxt_.map_frame_;
    local_path_.poses.clear();

    geometry_msgs::msg::PoseStamped pose_msg;
    for (auto &i : segments_) {
      pose_msg.pose = i->plan().fp.pose.pose.to_msg();
      local_path_.poses.push_back(pose_msg);
    }

    // Add last goal pose to path message
    pose_msg.pose = segments_.back()->goal().pose.pose.to_msg();
    local_path_.poses.push_back(pose_msg);

    if (keep_station_) {
      // Keep station at the last target
      add_keep_station_segment(plan, 1e6);
    }

    RCLCPP_INFO(logger_, "planned duration %g seconds", (plan.t - start.t).seconds());
    RCLCPP_INFO(logger_, "segment 1 of %d", segments_.size());
    segments_[0]->log_info();
  }

  void LocalPlanner::add_keep_station_segment(FPStamped &plan, double seconds)
  {
    segments_.push_back(std::make_shared<Pause>(logger_, cxt_, plan, rclcpp::Duration::from_seconds(seconds)));
  }

  void LocalPlanner::add_vertical_segment(FPStamped &plan, double z)
  {
    segments_.push_back(TrapVelo::make_vertical(logger_, cxt_, plan, z));
  }

  void LocalPlanner::add_rotate_segment(FPStamped &plan, double yaw)
  {
    segments_.push_back(TrapVelo::make_rotate(logger_, cxt_, plan, yaw));
  }

  void LocalPlanner::add_line_segment(FPStamped &plan, double x, double y)
  {
    segments_.push_back(TrapVelo::make_line(logger_, cxt_, plan, x, y));
  }

  void LocalPlanner::add_pose_segment(FPStamped &plan, const FP &goal)
  {
    segments_.push_back(TrapVelo::make_pose(logger_, cxt_, plan, goal));
  }

  bool LocalPlanner::advance(const rclcpp::Duration &d, FPStamped &plan, const FPStamped &estimate,
                             orca::Pose &error, orca::Efforts &efforts,
                             const std::function<void(double completed, double total)> &send_feedback)
  {
    // Update the plan
    if (segments_[segment_idx_]->advance(d)) {
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
    controller_->calc(d, plan.fp, estimate.fp, segments_[segment_idx_]->ff(), error, efforts);

    return true;
  }

  //=====================================================================================
  // MoveToMarkerPlanner -- a recovery strategy
  //=====================================================================================

  MoveToMarkerPlanner::MoveToMarkerPlanner(const rclcpp::Logger &logger, const BaseContext &cxt,
                                           const ObservationStamped &start) :
    logger_{logger}, cxt_{cxt}, marker_id_{start.o.id}, segment_idx_{0},
    controller_{std::make_shared<ObservationController>(cxt_)}
  {
    // Start observation
    ObservationStamped plan = start;

    // Goal observation
    Observation goal;
    goal = start.o;

    // Rotate to face the marker
    goal.yaw = 0;
    segments_.push_back(std::make_shared<RotateToMarker>(logger_, cxt_, plan, goal));
    plan.o = goal;

    // Move toward the marker
    goal.distance = 1;
    segments_.push_back(std::make_shared<MoveToMarker>(logger_, cxt_, plan, goal));

    // Start
    segments_[segment_idx_]->log_info();
  }

  bool MoveToMarkerPlanner::advance(const rclcpp::Duration &d, FPStamped &plan, const FPStamped &estimate,
                                    orca::Pose &error, orca::Efforts &efforts,
                                    const std::function<void(double completed, double total)> &send_feedback)
  {
    // Advance the plan
    if (segments_[segment_idx_]->advance(d)) {
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
    plan.fp.observations.push_back(segments_[segment_idx_]->plan().o);

    // Run the PID controller(s) and calculate efforts
    // If marker was not observed, estimate.obs.id == NOT_A_MARKER, and calc() will ignore PID outputs
    Observation estimate_obs;
    estimate.fp.good_obs(marker_id_, estimate_obs);
    controller_->calc(d, segments_[segment_idx_]->plan().o, cxt_.auv_z_target_, estimate_obs, estimate.fp.pose.pose.z,
                      segments_[segment_idx_]->ff(), error, efforts);

    return true;
  }

  //=====================================================================================
  // GlobalPlanner
  //=====================================================================================

  GlobalPlanner::GlobalPlanner(const rclcpp::Logger &logger, const BaseContext &cxt, Map map,
                               orca_description::Parser parser, const image_geometry::PinholeCameraModel &fcam_model,
                               std::vector<Target> targets, bool keep_station) :
    logger_{logger}, cxt_{cxt}, map_{std::move(map)}, parser_{std::move(parser)}, fcam_model_{fcam_model},
    targets_{std::move(targets)}, keep_station_{keep_station}, target_idx_{0}
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
      pose_msg.pose = i.fp.pose.pose.to_msg();
      global_path_.poses.push_back(pose_msg);
    }
  }

  void GlobalPlanner::predict_observations(FP &plan) const
  {
    // Get t_fcam_map
    // This method works whether or not t_base_map is published

    geometry_msgs::msg::Pose base_f_map = plan.pose.pose.to_msg();

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
      if (marker.predict_observation(cxt_, fcam_model_, t_fcam_map, obs)) {
        ++num_observations;
        plan.observations.push_back(obs);
      }
    }

    // RCLCPP_INFO(logger_, "Predicted %d observation(s)", num_observations);
  }

  void GlobalPlanner::start_local_plan(const FPStamped &start)
  {
    recovery_planner_ = nullptr;

    local_planner_ = std::make_shared<LocalPlanner>(logger_, cxt_, start, targets_[target_idx_], map_,
                                                    target_idx_ == targets_.size() - 1 ? keep_station_ : false);
  }

  void GlobalPlanner::start_recovery_plan(const ObservationStamped &start)
  {
    local_planner_ = nullptr;

    recovery_planner_ = std::make_shared<MoveToMarkerPlanner>(logger_, cxt_, start);
  }

  int GlobalPlanner::advance(const rclcpp::Duration &d, FPStamped &plan, const FPStamped &estimate,
                             orca::Pose &error, orca::Efforts &efforts,
                             const std::function<void(double completed, double total)> &send_feedback)
  {
    // Bootstrap
    if (!local_planner_ && !recovery_planner_) {
      if (estimate.fp.good_pose(cxt_.good_pose_dist_)) {
        RCLCPP_INFO(logger_, "start local plan from good pose");
        start_local_plan(estimate);
      } else {
        ObservationStamped closest_observation;
        if (estimate.closest_obs(closest_observation) < cxt_.good_obs_dist_) {
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
      if (local_planner_->advance(d, plan, estimate, error, efforts, send_feedback)) {

        // Predict observations from the planned pose
        predict_observations(plan.fp);

        // Is there a good pose?
        if (estimate.fp.good_pose(cxt_.good_pose_dist_)) {

          // Good pose, make sure the plan & estimate are reasonably close
          if (estimate.distance_xy(plan) > cxt_.planner_max_pose_xy_error_) {
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
          Observation plan_target_obs;
          ObservationStamped estimate_target_obs;

          if (plan.fp.good_obs(targets_[target_idx_].marker_id, plan_target_obs) &&
              estimate.good_obs(targets_[target_idx_].marker_id, estimate_target_obs)) {

            double err_yaw = std::abs(plan_target_obs.yaw - estimate_target_obs.o.yaw);

            if (plan_target_obs.distance < cxt_.good_obs_dist_ &&
                estimate_target_obs.o.distance < cxt_.good_obs_dist_ &&
                err_yaw > cxt_.planner_max_obs_yaw_error_) {
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

          if (estimate.fp.good_pose(cxt_.good_pose_dist_)) {
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
      if (estimate.fp.good_pose(cxt_.good_pose_dist_)) {
        RCLCPP_INFO(logger_, "recovery succeeded");

        // Known good pose right now... re-plan immediately
        start_local_plan(estimate);
      } else {
        // Advance the recovery plan
        if (!recovery_planner_->advance(d, plan, estimate, error, efforts, send_feedback)) {
          RCLCPP_INFO(logger_, "recovery complete, no good pose, giving up");
          return AdvanceRC::FAILURE;
        }

        // plan is a FiducialPose with a single planned observation -- the marker we're following
        assert(plan.fp.observations.size() == 1);

        // Estimate the marker corners from the planned yaw and distance -- just for grins
        plan.fp.observations[0].estimate_corners(map_.marker_length(),
                                                 cxt_.fcam_hfov_, cxt_.fcam_hres_, cxt_.fcam_vres_);
      }
    }

    // If we got this far, continue the mission
    return AdvanceRC::CONTINUE;
  }

  // Floor == true: markers must be on the floor facing up, and there must be a down-facing camera
  // Floor == false: markers must be on the wall, and there must be a forward-facing camera
  Target marker_to_target(const BaseContext &cxt, const Marker &marker, bool floor)
  {
    Target target;
    target.marker_id = marker.id;
    target.fp.pose.pose.from_msg(marker.marker_f_map);

    // Set plan.z from parameters
    target.fp.pose.pose.z = cxt.auv_z_target_;

    if (!floor) {
      // Target is in front of the marker
      target.fp.pose.pose.x += sin(target.fp.pose.pose.yaw) * cxt.auv_xy_distance_;
      target.fp.pose.pose.y -= cos(target.fp.pose.pose.yaw) * cxt.auv_xy_distance_;

      // Face the marker to get a good pose
      target.fp.pose.pose.yaw = norm_angle(target.fp.pose.pose.yaw + M_PI_2);
    }

    return target;
  }

  std::shared_ptr<GlobalPlanner>
  GlobalPlanner::plan_markers(const rclcpp::Logger &logger, const BaseContext &cxt, const Map &map,
                              const orca_description::Parser &parser,
                              const image_geometry::PinholeCameraModel &fcam_model,
                              const std::vector<int> &markers_ids, bool random, bool repeat, bool keep_station)
  {
    // TODO repeat

    std::vector<Target> targets;

    if (markers_ids.empty()) {
      // Use all of the markers in the map
      for (const auto &marker : map.markers()) {
        targets.push_back(marker_to_target(cxt, marker, false));
      }
    } else {
      // Use just the markers in the list
      for (const auto marker_id : markers_ids) {
        Marker marker;
        if (map.find_marker(marker_id, marker)) {
          targets.push_back(marker_to_target(cxt, marker, false));
        } else {
          RCLCPP_WARN(logger, "marker %d is not in the map, skipping", marker_id);
        }
      }
    }

    if (targets.empty()) {
      RCLCPP_ERROR(logger, "no targets, abort mission");
      return nullptr;
    }

    if (random) {
      // Shuffle targets
      std::random_device rd;
      std::mt19937 g(rd());
      std::shuffle(targets.begin(), targets.end(), g);
    }

    return std::make_shared<GlobalPlanner>(logger, cxt, map, parser, fcam_model, targets, keep_station);
  }

  Target pose_to_target(const geometry_msgs::msg::Pose &pose)
  {
    Target target;
    target.marker_id = NOT_A_MARKER;
    target.fp.pose.pose.from_msg(pose);
    return target;
  }

  std::shared_ptr<GlobalPlanner>
  GlobalPlanner::plan_poses(const rclcpp::Logger &logger, const BaseContext &cxt, const Map &map,
                            const orca_description::Parser &parser,
                            const image_geometry::PinholeCameraModel &fcam_model,
                            const std::vector<geometry_msgs::msg::Pose> &poses, bool random, bool repeat,
                            bool keep_station)
  {
    // TODO repeat

    std::vector<Target> targets;

    if (poses.empty()) {
      RCLCPP_ERROR(logger, "no poses, abort mission");
      return nullptr;
    }

    for (auto pose : poses) {
      targets.push_back(pose_to_target(pose));
    }

    if (random) {
      // Shuffle targets
      std::random_device rd;
      std::mt19937 g(rd());
      std::shuffle(targets.begin(), targets.end(), g);
    }

    return std::make_shared<GlobalPlanner>(logger, cxt, map, parser, fcam_model, targets, keep_station);
  }

} // namespace orca_base
