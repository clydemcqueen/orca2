#ifndef ORCA_SHARED_BARO_HPP
#define ORCA_SHARED_BARO_HPP

#include "orca_shared/model.hpp"

namespace orca
{

  class Barometer
  {
    double atmospheric_pressure_{0};
    bool initialized_{false};

  public:

    bool initialized() const
    { return initialized_; }

    // Given a pressure, return base_link.z
    // The first reading must be air pressure at the surface
    double pressure_to_base_link_z(const Model &model, double pressure);
  };

} // namespace orca

#endif //ORCA_SHARED_BARO_HPP