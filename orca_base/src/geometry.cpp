#include "orca_base/geometry.hpp"

namespace orca_base
{

  std::ostream &operator<<(std::ostream &os, Pose const &pose)
  {
    return os << "{" << pose.x << ", " << pose.y << ", " << pose.z << ", " << pose.yaw << "}";
  }

} // namespace orca_base