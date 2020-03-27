#ifndef ORCA_FILTER_FILTER_CONTEXT_HPP
#define ORCA_FILTER_FILTER_CONTEXT_HPP

#include <cmath>
#include <string>
#include <vector>

#include "ros2_shared/context_macros.hpp"

#include "orca_shared/model.hpp"

namespace orca_filter
{

#define FILTER_NODE_ALL_PARAMS \
  CXT_MACRO_MEMBER(param_fluid_density, double, 997)          /* kg/m^3, 997 for freshwater, 1029 for seawater  */ \
  \
  CXT_MACRO_MEMBER(timeout_open_water_ms, int, 300)           /* Flip to a depth filter when we hit open water  */ \
  CXT_MACRO_MEMBER(timeout_outlier_ms, int, 300)              /* Reset the filter if poses are consistently rejected as outliers  */ \
  \
  CXT_MACRO_MEMBER(frame_id_map, std::string, "map") \
  CXT_MACRO_MEMBER(frame_id_base_link, std::string, "base_link") \
  \
  CXT_MACRO_MEMBER(publish_tf, bool, "true")                  /* Publish t_map_base  */ \
  \
  CXT_MACRO_MEMBER(predict_accel, bool, true)                 /* Predict acceleration  */ \
  CXT_MACRO_MEMBER(predict_accel_control, bool, true)         /* Add u_bar to predicted acceleration  */ \
  CXT_MACRO_MEMBER(predict_accel_drag, bool, true)            /* Add drag to predicted acceleration  */ \
  CXT_MACRO_MEMBER(predict_accel_buoyancy, bool, true)        /* Add gravity and buoyancy to predicted acceleration  */ \
  \
  CXT_MACRO_MEMBER(filter_baro, bool, true)                   /* Filter barometer messages  */ \
  CXT_MACRO_MEMBER(filter_fcam, bool, false)                  /* Filter forward camera messages  */ \
  \
  CXT_MACRO_MEMBER(good_pose_dist, double, 1.8)               /* Good pose if marker < 1.8m away  */ \
  CXT_MACRO_MEMBER(good_obs_dist, double, 10)                 /* Good observation if marker < 10m away  */ \
  CXT_MACRO_MEMBER(outlier_distance, double, 4.0)             /* Reject measurements > n std devs from estimate  */ \
  \
  CXT_MACRO_MEMBER(four_dof, bool, false)                     /* Experiment: run 4dof filter instead of 6dof filter  */ \
  CXT_MACRO_MEMBER(min_dt, double, 0.001)                     /* Minimum dt between message timestamps  */ \
  CXT_MACRO_MEMBER(max_dt, double, 0.3)                       /* Maxiumum dt between message timestamps  */ \
  CXT_MACRO_MEMBER(default_dt, double, 0.1)                   /* If dt > max, use default instead  */ \
  CXT_MACRO_MEMBER(always_publish_odom, bool, false)          /* False: don't publish odom from depth messages in a pose filter  */ \
  \
  CXT_MACRO_MEMBER(ukf_alpha, double, 0.001)                  /* UKF alpha -- see UKF  */ \
  CXT_MACRO_MEMBER(ukf_beta, double, 2.0)                     /* UKF beta -- see UKF  */ \
  CXT_MACRO_MEMBER(ukf_kappa, int, 0)                         /* UKF kappa -- see UKF  */ \
/* End of list */

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)

  struct FilterContext
  {
    FILTER_NODE_ALL_PARAMS

    // Orca model
    orca::Model model_{};
  };

} // namespace orca_filter

#endif // ORCA_FILTER_FILTER_CONTEXT_HPP
