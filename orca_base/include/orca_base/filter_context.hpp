#ifndef ORCA_BASE_FILTER_CONTEXT_HPP
#define ORCA_BASE_FILTER_CONTEXT_HPP

#include <cmath>
#include <string>
#include <vector>

#include "ros2_shared/context_macros.hpp"

#include "orca_base/model.hpp"

namespace orca_base
{

#define FILTER_NODE_ALL_PARAMS \
  CXT_MACRO_MEMBER(param_fluid_density, double, 997)          /* kg/m^3, 997 for freshwater, 1029 for seawater  */ \
  \
  CXT_MACRO_MEMBER(urdf_file, std::string, "install/orca_description/share/orca_description/urdf/orca.urdf") \
  CXT_MACRO_MEMBER(urdf_barometer_joint, std::string, "baro_joint")  /* Joint between baro_link and base_link */ \
  CXT_MACRO_MEMBER(urdf_forward_camera_joint, std::string, "forward_camera_frame_joint") \
  CXT_MACRO_MEMBER(urdf_left_camera_joint, std::string, "left_camera_frame_joint") \
  CXT_MACRO_MEMBER(urdf_right_camera_joint, std::string, "right_camera_frame_joint") \
  \
  CXT_MACRO_MEMBER(frame_id_map, std::string, "map") \
  CXT_MACRO_MEMBER(frame_id_base_link, std::string, "base_link") \
  \
  CXT_MACRO_MEMBER(publish_filtered_tf, bool, "true") \
  CXT_MACRO_MEMBER(publish_measurement_tf, bool, "true") \
  \
  CXT_MACRO_MEMBER(baro_init, int, 0)                         /* Barometer init method: 0 in air, 1 in water, 2 from map  */ \
  \
  CXT_MACRO_MEMBER(predict_accel, bool, true)                 /* Predict acceleration  */ \
  CXT_MACRO_MEMBER(predict_accel_control, bool, true)         /* Add u_bar to predicted acceleration  */ \
  CXT_MACRO_MEMBER(predict_accel_drag, bool, true)            /* Add drag to predicted acceleration  */ \
  CXT_MACRO_MEMBER(predict_accel_buoyancy, bool, true)        /* Add gravity and buoyancy to predicted acceleration  */ \
  \
  CXT_MACRO_MEMBER(filter_baro, bool, false)                  /* Filter barometer messages  */ \
  CXT_MACRO_MEMBER(filter_fcam, bool, true)                   /* Filter forward camera messages  */ \
  CXT_MACRO_MEMBER(filter_lcam, bool, true)                   /* Filter left camera messages  */ \
  CXT_MACRO_MEMBER(filter_rcam, bool, true)                   /* Filter right camera messages  */ \
  \
  CXT_MACRO_MEMBER(outlier_distance, double, 10.0)            /* Reject measurements > n std devs from estimate  */ \
  \
  CXT_MACRO_MEMBER(four_dof, bool, false)                     /* Experiment: run 4dof filter instead of 6dof filter  */ \
/* End of list */

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)

  struct FilterContext
  {
    FILTER_NODE_ALL_PARAMS

    // Orca model
    Model model_{};
  };

} // namespace orca_base

#endif // ORCA_BASE_FILTER_CONTEXT_HPP
