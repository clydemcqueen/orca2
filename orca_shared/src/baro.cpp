#include "orca_shared/baro.hpp"

#include <iostream>

/***************************************************************
 * How is orca::Barometer initialized?
 *
 * Start the system (orca_driver, auv_node) while the sub is in the air
 * orca_driver publishes raw pressure (no adjustments) on /barometer
 * auv_node subscribes to /barometer, passes the first pressure reading to orca::Barometer
 * orca::Barometer saves the 1st reading as atmospheric_pressure_
 *
 * What about simulation?
 *
 * For 2s BarometerPlugin sends ATMOSPHERIC_PRESSURE
 * auv_node passes the first reading to orca::Barometer, etc.
 * The model should be injected into the simulation at {0, 0, 0}
 * See notes in orca_gazebo::BarometerPlugin for more information
 *
 * When the sub is floating at the surface of the water:
 * base_link_z ~= -0.125
 * baro_link_z ~= -0.075
 */

namespace orca
{
  // TODO get from urdf or tf tree
  static const double baro_link_to_base_link_z = -0.05;

  double Barometer::pressure_to_base_link_z(const Model &model, double pressure)
  {
    // First reading is atmospheric pressure
    if (!initialized_) {
      atmospheric_pressure_ = pressure;
      initialized_ = true;
      std::cout << "atmospheric pressure is " << atmospheric_pressure_ << std::endl;
    }

    // Calc depth from pressure
    double baro_link_z = model.pressure_to_z(atmospheric_pressure_, pressure);

    // Transform baro_link to base_link
    double base_link_z = baro_link_z + baro_link_to_base_link_z;

    return base_link_z;
  }
}