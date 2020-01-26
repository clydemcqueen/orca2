#ifndef ORCA_SHARED_BARO_HPP
#define ORCA_SHARED_BARO_HPP

#include "orca_shared/model.hpp"

namespace orca
{

  class Barometer
  {
    // Three ways to initialize the barometer:
    // init_method_ == 0:   Orca is in the air, so the first z reading is just air pressure
    // init_method_ == 1:   Orca is floating at the surface in the water, the barometer is submerged ~5cm
    // init_method_ == 2:   Future: wait for good odometry from fiducial_vlam and initialize barometer from the map
    int init_method_;

    bool initialized_{false};

    double offset_{0};

  public:

    Barometer(int init_method) : init_method_{init_method}
    {}

    double pressure_to_depth(const Model &model, double pressure);
  };

} // namespace orca

#endif //ORCA_SHARED_BARO_HPP