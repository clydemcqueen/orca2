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
  CXT_MACRO_MEMBER(model_fluid_density, double, 997)          /* kg/m^3, 997 for freshwater, 1029 for seawater  */ \
  \
  CXT_MACRO_MEMBER(baro_init, int, 0)                         /* Barometer init method: 0 in air, 1 in water, 2 from map  */ \
  \
  CXT_MACRO_MEMBER(filter_predict_accel, bool, true)          /* Predict acceleration  */ \
  CXT_MACRO_MEMBER(filter_predict_control, bool, true)        /* Add u_bar to predicted acceleration  */ \
  CXT_MACRO_MEMBER(filter_predict_drag, bool, true)           /* Add drag to predicted acceleration  */ \
  CXT_MACRO_MEMBER(filter_baro, bool, false)                  /* Add barometer messages to the filter  */ \
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
