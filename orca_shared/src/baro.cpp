#include "orca_shared/baro.hpp"

#include <iostream>

namespace orca
{
  // TODO pull these from the URDF
  static const double z_top_to_baro_link = -0.05;
  static const double z_baro_link_to_base_link = -0.085;

  double Barometer::pressure_to_depth(const Model &model, double pressure)
  {
    // Calc depth from pressure -- depends on water density
    double z = model.pressure_to_z(pressure);

    // Initialize offset
    if (!initialized_ && init_method_ == 0) {

      offset_ = -z + z_top_to_baro_link + z_baro_link_to_base_link;
      initialized_ = true;
      std::cout << "barometer init mode 0 (in air): adjustment " << offset_ << std::endl;

    } else if (!initialized_ && init_method_ == 1) {

      offset_ = -z + z_baro_link_to_base_link;
      initialized_ = true;
      std::cout << "barometer init mode 1 (in water): adjustment " << offset_ << std::endl;

    } else if (!initialized_) {

      std::cout << "can't initialize barometer!" << std::endl;

    }

    // Return anyway
    return z + offset_;
  }
}