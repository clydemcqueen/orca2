#include "orca_base/global_planner.hpp"

#include <random>

#include "orca_msgs/msg/control.hpp"
#include "orca_shared/util.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace orca;

namespace orca_base
{

  GlobalPlanner::GlobalPlanner(const rclcpp::Logger &logger, const AUVContext &cxt, Map map,
                               orca_description::Parser parser,
                               const image_geometry::PinholeCameraModel &fcam_model, std::vector<Target> targets,
                               bool keep_station) :
    logger_{logger}, cxt_{cxt}, map_{std::move(map)}, parser_{std::move(parser)}, fcam_model_{fcam_model},
    targets_{std::move(targets)}, keep_station_{keep_station}
  {
    RCLCPP_INFO_STREAM(logger_, targets_.size() << " targets:");
    for (auto &i : targets_) {
      RCLCPP_INFO_STREAM(logger_, i);
    }

    // Init status
    status_.first_target(targets_.size(), targets_[0].marker_id);

    // Write global_path_
    global_path_.header.frame_id = cxt_.map_frame_;
    global_path_.poses.clear();
    geometry_msgs::msg::PoseStamped pose_msg;
    for (auto &i : targets_) {
      pose_msg.pose = i.fp.pose.pose.to_msg();
      global_path_.poses.push_back(pose_msg);
    }
  }

  // TODO move this to map.cpp
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

    auto keep_station = status_.target_idx == targets_.size() - 1 ? keep_station_ : false;
    local_planner_ = std::make_shared<LocalPlanner>(logger_, cxt_, start, targets_[status_.target_idx], map_,
                                                    keep_station, status_);
  }

  void GlobalPlanner::start_recovery_plan(const ObservationStamped &start)
  {
    local_planner_ = nullptr;

    recovery_planner_ = std::make_shared<MoveToMarkerPlanner>(logger_, cxt_, start, status_);
  }

  int GlobalPlanner::advance(const rclcpp::Duration &d, const FPStamped &estimate, orca::Efforts &efforts,
                             const std::function<void(double completed, double total)> &send_feedback)
  {
    // Bootstrap
    if (!local_planner_ && !recovery_planner_) {
      if (estimate.fp.good_pose(cxt_.good_pose_dist_)) {
        RCLCPP_INFO(logger_, "start local plan from good pose");
        start_local_plan(estimate);
      } else {
        ObservationStamped closest_observation;
        if (estimate.get_closest_observation(closest_observation) < cxt_.good_obs_dist_) {
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
      if (local_planner_->advance(d, estimate, efforts, status_)) {

        // Predict observations from the planned pose
        predict_observations(status_.pose.fp);

        // Is there a good pose?
        if (estimate.fp.good_pose(cxt_.good_pose_dist_)) {

          // Good pose, make sure the plan & estimate are reasonably close
          if (estimate.distance_xy(status_.pose) > cxt_.planner_max_pose_xy_error_) {
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
          auto target_id = targets_[status_.target_idx].marker_id;

          if (status_.pose.fp.get_good_observation(cxt_.good_obs_dist_, target_id, plan_target_obs) &&
              estimate.get_good_observation(cxt_.good_obs_dist_, target_id, estimate_target_obs)) {

            if (std::abs(plan_target_obs.yaw - estimate_target_obs.o.yaw) > cxt_.planner_max_obs_yaw_error_) {
              RCLCPP_INFO(logger_, "poor pose, target marker in view but yaw error is high, recover");
              start_recovery_plan(estimate_target_obs);
              return AdvanceRC::CONTINUE;
            }
          }
        }

      } else {

        // Local plan is complete
        // Move to next target
        if (++status_.target_idx < targets_.size()) {
          RCLCPP_INFO_STREAM(logger_, "next target: " << targets_[status_.target_idx]);
          send_feedback(status_.target_idx, targets_.size());

          // Update status
          status_.next_target(targets_[status_.target_idx].marker_id);

          // Build a local plan
          start_local_plan(estimate);
        } else {
          // Mission complete!
          return AdvanceRC::SUCCESS;
        }
      }

    } else {

#ifdef STOP_AT_GOOD_POSE
      // In recovery. Success?
      if (estimate.fp.good_pose(cxt_.good_pose_dist_)) {
        RCLCPP_INFO(logger_, "recovery succeeded");

        // Known good pose right now... re-plan immediately
        start_local_plan(estimate);
      } else {
        // Advance the recovery plan
        if (!recovery_planner_->advance(d, estimate, efforts, status_)) {
          RCLCPP_INFO(logger_, "recovery complete, no good pose, giving up");
          return AdvanceRC::FAILURE;
        }

        // status_.pose.fp is a FiducialPose with a single planned observation -- the marker we're following
        assert(status_.pose.fp.observations.size() == 1);

        // Estimate the marker corners from the planned yaw and distance -- just for grins
        status_.pose.fp.observations[0].estimate_corners(map_.marker_length(),
                                                         cxt_.fcam_hfov_, cxt_.fcam_hres_, cxt_.fcam_vres_);
      }
#else
      // Advance the recovery plan until it completes
      // The controller will happily run w/o observations -- a bit like dead reckoning
      // It's reasonable to miss a few observations, but not to lose the marker completely TODO add a timeout
      if (recovery_planner_->advance(d, estimate, efforts, status_)) {
        // status_.pose.fp is a FiducialPose with a single planned observation -- the marker we're following
        assert(status_.pose.fp.observations.size() == 1);

        // Estimate the marker corners from the planned yaw and distance -- just for grins
        status_.pose.fp.observations[0].estimate_corners(map_.marker_length(),
                                                         cxt_.fcam_hfov_, cxt_.fcam_hres_, cxt_.fcam_vres_);
      } else {
        if (estimate.fp.good_pose(cxt_.good_pose_dist_)) {
          RCLCPP_INFO(logger_, "recovery succeeded");

          // Known good pose right now... re-plan immediately
          start_local_plan(estimate);
        } else {
          RCLCPP_WARN(logger_, "recovery complete, no good pose, giving up");
          return AdvanceRC::FAILURE;
        }
      }
#endif
    }

    // If we got this far, continue the mission
    return AdvanceRC::CONTINUE;
  }

  // Floor == true: markers must be on the floor facing up, and there must be a down-facing camera
  // Floor == false: markers must be on the wall, and there must be a forward-facing camera
  Target marker_to_target(const AUVContext &cxt, const Marker &marker, bool floor)
  {
    Target target;
    target.marker_id = marker.id;
    target.fp.pose.pose.from_msg(marker.marker_f_map);

    // Set plan.z from parameters
    target.fp.pose.pose.z = cxt.planner_target_z_;

    if (!floor) {
      // Target is in front of the marker
      target.fp.pose.pose.x += sin(target.fp.pose.pose.yaw) * cxt.local_planner_target_dist_;
      target.fp.pose.pose.y -= cos(target.fp.pose.pose.yaw) * cxt.local_planner_target_dist_;

      // Face the marker to get a good pose
      target.fp.pose.pose.yaw = norm_angle(target.fp.pose.pose.yaw + M_PI_2);
    }

    return target;
  }

  std::shared_ptr<GlobalPlanner>
  GlobalPlanner::plan_markers(const rclcpp::Logger &logger, const AUVContext &cxt, const Map &map,
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
  GlobalPlanner::plan_poses(const rclcpp::Logger &logger, const AUVContext &cxt, const Map &map,
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
