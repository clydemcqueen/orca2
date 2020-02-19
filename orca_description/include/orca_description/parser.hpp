#ifndef ORCA_DESCRIPTION_PARSER_HPP
#define ORCA_DESCRIPTION_PARSER_HPP

#include <string>

#include "tf2/LinearMath/Transform.h"
#include "urdf/model.h"

namespace orca_description
{
  constexpr const char *filename = "install/orca_description/share/orca_description/urdf/orca.urdf";
  constexpr const char *base_link = "base_link";
  constexpr const char *barometer_joint = "baro_joint";

  // Note that these are x_camera_FRAME_joint, not x_camera_joint
  constexpr const char *forward_camera_joint = "forward_camera_frame_joint";
  constexpr const char *left_camera_joint = "left_camera_frame_joint";
  constexpr const char *right_camera_joint = "right_camera_frame_joint";

  struct Parser
  {
    // Transform base_f_sensor_frame for all sensors
    tf2::Transform t_baro_base{};
    tf2::Transform t_fcam_base{};
    tf2::Transform t_lcam_base{};
    tf2::Transform t_rcam_base{};

    // Parse URDF and set t_foo_base, return true if OK
    bool parse();

    // Get a particular joint, return true if OK
    static bool get_joint(const urdf::Model &model, const std::string &joint_name, tf2::Transform &t);
  };

} // namespace orca_description

#endif //ORCA_DESCRIPTION_PARSER_HPP
