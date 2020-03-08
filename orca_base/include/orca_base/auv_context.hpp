#ifndef ORCA_BASE_AUV_CONTEXT_HPP
#define ORCA_BASE_AUV_CONTEXT_HPP

#include <cmath>
#include <string>
#include <vector>

#include "ros2_shared/context_macros.hpp"

#include "orca_shared/model.hpp"

namespace orca_base
{

#define AUV_NODE_ALL_PARAMS \
  CXT_MACRO_MEMBER(bollard_force_xy, double, 137)             /* Newtons, bollard force xy or fs  */ \
  CXT_MACRO_MEMBER(bollard_force_z, double, 88)               /* Newtons, bollard force z  */ \
  CXT_MACRO_MEMBER(t200_max_pos_force, double, 50)            /* Newtons, max positive force for T200 thruster  */ \
  CXT_MACRO_MEMBER(t200_max_neg_force, double, 44)            /* Newtons, max negative force for T200 thruster  */ \
  CXT_MACRO_MEMBER(fluid_density, double, 997)                /* kg/m^3, 997 for freshwater, 1029 for seawater  */ \
  CXT_MACRO_MEMBER(drag_coef_f, double, 0.8)                  /* Forward drag, 1.0 is a box  */ \
  CXT_MACRO_MEMBER(drag_coef_s, double, 0.95)                 /* Strafe drag  */ \
  CXT_MACRO_MEMBER(drag_coef_z, double, 0.95)                 /* Vertical drag  */ \
  CXT_MACRO_MEMBER(drag_coef_tether, double, 1.1)             /* Tether drag, 1.2 for unfaired tether  */ \
  CXT_MACRO_MEMBER(drag_partial_const_yaw, double, 0.004)     /* Yaw drag, wild guess  */ \
  \
  CXT_MACRO_MEMBER(loop_driver, int, 0)                       /* What drives auv_advance? 0: timer, 1: depth msg, 2: fiducial msg  */ \
  CXT_MACRO_MEMBER(filtered_odom, bool, false)                /* Subscribe to filtered odom msg, vs raw pose  */ \
  CXT_MACRO_MEMBER(fuse_depth, bool, true)                    /* Fuse depth and fiducial messages  */ \
  CXT_MACRO_MEMBER(timer_period_ms, int, 50)                  /* Timer period in ms  */ \
  CXT_MACRO_MEMBER(timeout_depth_ms, int, 250)                /* Depth message timeout in ms  */ \
  CXT_MACRO_MEMBER(timeout_fp_ms, int, 250)                   /* Fiducial pose message timeout in ms  */ \
  CXT_MACRO_MEMBER(keep_poses, int, 500)                      /* Max # of poses on filtered_path  */ \
  \
  CXT_MACRO_MEMBER(publish_tf, bool, false)                   /* Publish t_map_base  */ \
  CXT_MACRO_MEMBER(map_frame, std::string, "map")             /* Map frame  */ \
  CXT_MACRO_MEMBER(base_frame, std::string, "base_link")      /* Base frame  */ \
  \
  CXT_MACRO_MEMBER(fcam_hfov, double, 1.4)                    /* Forward camera horiz field of view in radians  */ \
  CXT_MACRO_MEMBER(fcam_hres, double, 800)                    /* Forward camera horiz resolution in pixels  */ \
  CXT_MACRO_MEMBER(fcam_vres, double, 600)                    /* Forward camera vertical resolution in pixels  */ \
  \
  CXT_MACRO_MEMBER(xy_gain, double, 0.5)                      /* Limit fwd/strafe motion, rest is yaw  */ \
  \
  CXT_MACRO_MEMBER(auv_x_pid_kp, double, 0.6)                 /* AUV x pid Kp  */ \
  CXT_MACRO_MEMBER(auv_x_pid_ki, double, 0.2)                 /* AUV x pid Ki  */ \
  CXT_MACRO_MEMBER(auv_x_pid_kd, double, 0.45)                /* AUV x pid Kd  */ \
  \
  CXT_MACRO_MEMBER(auv_y_pid_kp, double, 0.6)                 /* AUV y pid Kp  */ \
  CXT_MACRO_MEMBER(auv_y_pid_ki, double, 0.2)                 /* AUV y pid Ki  */ \
  CXT_MACRO_MEMBER(auv_y_pid_kd, double, 0.45)                /* AUV y pid Kd  */ \
  \
  CXT_MACRO_MEMBER(auv_z_pid_kp, double, 0.6)                 /* AUV z pid Kp  */ \
  CXT_MACRO_MEMBER(auv_z_pid_ki, double, 0.2)                 /* AUV z pid Ki  */ \
  CXT_MACRO_MEMBER(auv_z_pid_kd, double, 0.45)                /* AUV z pid Kd  */ \
  \
  CXT_MACRO_MEMBER(auv_yaw_pid_kp, double, 0.6)               /* AUV yaw pid Kp  */ \
  CXT_MACRO_MEMBER(auv_yaw_pid_ki, double, 0.2)               /* AUV yaw pid Ki  */ \
  CXT_MACRO_MEMBER(auv_yaw_pid_kd, double, 0.45)              /* AUV yaw pid Kd  */ \
  \
  CXT_MACRO_MEMBER(mtm_fwd_pid_kp, double, 0.4)               /* Move to marker forward pid Kp  */ \
  CXT_MACRO_MEMBER(mtm_fwd_pid_ki, double, 0.0)               /* Move to marker forward pid Ki  */ \
  CXT_MACRO_MEMBER(mtm_fwd_pid_kd, double, 0.0)               /* Move to marker forward pid Kd  */ \
  \
  CXT_MACRO_MEMBER(auv_xy_accel, double, 0.2)                 /* AUV acceleration in the xy plane  */ \
  CXT_MACRO_MEMBER(auv_xy_velo, double, 0.4)                  /* AUV velocity in the xy plane  */ \
  CXT_MACRO_MEMBER(auv_z_accel, double, 0.15)                 /* AUV vertical acceleration  */ \
  CXT_MACRO_MEMBER(auv_z_velo, double, 0.3)                   /* AUV vertical velocity  */ \
  CXT_MACRO_MEMBER(auv_yaw_accel, double, 0.2)                /* AUV rotation acceleration  */ \
  CXT_MACRO_MEMBER(auv_yaw_velo, double, 0.4)                 /* AUV rotation velocity  */ \
  \
  CXT_MACRO_MEMBER(mtm_fwd_accel, double, 0.2)                /* Move to marker forward acceleration  */ \
  CXT_MACRO_MEMBER(mtm_fwd_velo, double, 0.4)                 /* Move to marker forward velocity  */ \
  CXT_MACRO_MEMBER(mtm_yaw_accel, double, 0.1)                /* Move to marker yaw acceleration  */ \
  CXT_MACRO_MEMBER(mtm_yaw_velo, double, 0.2)                 /* Move to marker yaw velocity  */ \
  \
  CXT_MACRO_MEMBER(auv_epsilon_xy, double, 0.1)               /* Deadzone controller epsilon xy  */ \
  CXT_MACRO_MEMBER(auv_epsilon_z, double, 0.1)                /* Deadzone controller epsilon z  */ \
  CXT_MACRO_MEMBER(auv_epsilon_yaw, double, 0.2)              /* Deadzone controller epsilon yaw  */ \
  CXT_MACRO_MEMBER(auv_jerk_xy, double, 0.1)                  /* Slow controller jerk xy  */ \
  CXT_MACRO_MEMBER(auv_jerk_z, double, 0.1)                   /* Slow controller jerk z  */ \
  CXT_MACRO_MEMBER(auv_jerk_yaw, double, 0.2)                 /* Slow controller jerk yaw  */ \
  \
  CXT_MACRO_MEMBER(good_pose_dist, double, 1.8)               /* Good pose if marker < 1.8m away  */ \
  CXT_MACRO_MEMBER(good_obs_dist, double, 10)                 /* Good observation if marker < 10m away  */ \
  \
  CXT_MACRO_MEMBER(planner_look_for_waypoints, bool, false)   /* Use A* to find waypoints  */ \
  CXT_MACRO_MEMBER(planner_epsilon_xyz, double, 0.05)         /* If xyz distance < epsilon, skip this segment  */ \
  CXT_MACRO_MEMBER(planner_max_pose_xy_error, double, 0.6)    /* Replan if xy distance > max  */ \
  CXT_MACRO_MEMBER(planner_max_short_plan_xy, double, 2)      /* Build a long plan if xy distance > max  */ \
  CXT_MACRO_MEMBER(planner_max_obs_yaw_error, double, 0.2)    /* Start recovery if observation yaw error is > max  */ \
  CXT_MACRO_MEMBER(planner_max_dead_reckon_dist, double, 9)   /* Max allowable dead reckoning distance  */ \
  CXT_MACRO_MEMBER(planner_target_z, double, -0.5)            /* Target z position  */ \
  \
  CXT_MACRO_MEMBER(local_planner_target_dist, double, 1)      /* Local planner: target distance in front of marker  */ \
  \
  CXT_MACRO_MEMBER(mtm_planner_target_dist, double, 1)        /* MTM planner: target distance from marker */ \
/* End of list */

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)

  struct AUVContext
  {
    AUV_NODE_ALL_PARAMS

    // Orca model
    orca::Model model_{};
  };

} // namespace orca_base

#endif //ORCA_BASE_AUV_CONTEXT_HPP
