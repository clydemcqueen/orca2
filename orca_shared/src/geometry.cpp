#include "orca_shared/geometry.hpp"

namespace orca
{

  std::ostream &operator<<(std::ostream &os, Pose const &pose)
  {
    return os << "{" << pose.x << ", " << pose.y << ", " << pose.z << ", " << pose.yaw << "}";
  }

} // namespace orca_shared