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
  CXT_MACRO_MEMBER(fluid_density, double, 997)                /* kg/m^3, 997 for freshwater, 1029 for seawater  */ \
  CXT_MACRO_MEMBER(drag_coef_f, double, 0.8)                  /* Forward drag, 1.0 is a box  */ \
  CXT_MACRO_MEMBER(drag_coef_s, double, 0.95)                 /* Strafe drag  */ \
  CXT_MACRO_MEMBER(drag_coef_z, double, 0.95)                 /* Vertical drag  */ \
  CXT_MACRO_MEMBER(drag_coef_tether, double, 1.1)             /* Tether drag, 1.2 for unfaired tether  */ \
  CXT_MACRO_MEMBER(drag_partial_const_yaw, double, 0.002)     /* Yaw drag, wild guess  */ \
  \
  CXT_MACRO_MEMBER(sensor_loop, bool, false)                  /* false: timer loop, true: sensor loop  */ \
  CXT_MACRO_MEMBER(publish_tf, bool, false)                   /* true: publish t_map_base  */ \
  \
  CXT_MACRO_MEMBER(map_frame, std::string, "map")             /* Map frame  */ \
  CXT_MACRO_MEMBER(base_frame, std::string, "base_link")      /* Base frame  */ \
  CXT_MACRO_MEMBER(fcam_hfov, double, 1.4)                    /* Forward camera horiz field of view in radians  */ \
  CXT_MACRO_MEMBER(fcam_hres, double, 800)                    /* Forward camera horiz resolution in pixels  */ \
  CXT_MACRO_MEMBER(fcam_vres, double, 600)                    /* Forward camera vertical resolution in pixels  */ \
  \
  CXT_MACRO_MEMBER(xy_gain, double, 0.5)                      /* Limit fwd/strafe motion, rest is yaw  */ \
  \
  CXT_MACRO_MEMBER(auv_x_pid_ku, double, 1)                   /* AUV x pid Ziegler–Nichols ku  */ \
  CXT_MACRO_MEMBER(auv_x_pid_tu, double, 6)                   /* AUV x pid Ziegler–Nichols tu  */ \
  \
  CXT_MACRO_MEMBER(auv_y_pid_ku, double, 1)                   /* AUV y pid Ziegler–Nichols ku  */ \
  CXT_MACRO_MEMBER(auv_y_pid_tu, double, 6)                   /* AUV y pid Ziegler–Nichols tu  */ \
  \
  CXT_MACRO_MEMBER(auv_z_pid_ku, double, 1)                   /* AUV z pid Ziegler–Nichols ku  */ \
  CXT_MACRO_MEMBER(auv_z_pid_tu, double, 6)                   /* AUV z pid Ziegler–Nichols tu  */ \
  \
  CXT_MACRO_MEMBER(auv_yaw_pid_ku, double, 1)                 /* AUV yaw pid Ziegler–Nichols ku  */ \
  CXT_MACRO_MEMBER(auv_yaw_pid_tu, double, 6)                 /* AUV yaw pid Ziegler–Nichols tu  */ \
  \
  CXT_MACRO_MEMBER(auv_z_target, double, -0.5)                /* AUV path target z position  */ \
  CXT_MACRO_MEMBER(auv_xy_distance, double, 1)                /* AUV distance in front of marker  */ \
  \
  CXT_MACRO_MEMBER(auv_xy_accel, double, 0.25)                /* AUV acceleration in the xy plane  */ \
  CXT_MACRO_MEMBER(auv_xy_velo, double, 0.5)                  /* AUV velocity in the xy plane  */ \
  CXT_MACRO_MEMBER(auv_z_accel, double, 0.15)                 /* AUV vertical acceleration  */ \
  CXT_MACRO_MEMBER(auv_z_velo, double, 0.3)                   /* AUV vertical velocity  */ \
  CXT_MACRO_MEMBER(auv_yaw_accel, double, M_PI_4 / 4)         /* AUV rotation acceleration  */ \
  CXT_MACRO_MEMBER(auv_yaw_velo, double, M_PI_4 / 2)          /* AUV rotation velocity  */ \
  \
  CXT_MACRO_MEMBER(keep_poses, int, 500)                      /* Max # of poses on filtered_path  */ \
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
  \
  CXT_MACRO_MEMBER(timeout_baro_ms, int, 400)                 /* Barometer message timeout in ms  */ \
  CXT_MACRO_MEMBER(timeout_fp_ms, int, 200)                   /* Fiducial pose message timeout in ms  */ \
  CXT_MACRO_MEMBER(timer_period_ms, int, 50)                  /* Timer period in ms  */ \

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
