#include "orca_base/pose_segment.hpp"

#include <iostream>

#include "orca_shared/util.hpp"

#define CLOSE_ENOUGH(x) (abs(x) < 0.001)

void segment_test()
{
  std::cout << "=== TRAP2 SEGMENT TEST ===" << std::endl;

  mw::PoseStamped p0, p1, p2, p3;

  p3.pose().position().x() = -5;
  p3.pose().position().y() = 0;
  p3.pose().position().z() = 1;
  p3.pose().orientation().yaw(2);

  auto p3_save = p3;

  orca_base::AUVContext cxt{};

  mw::Acceleration a;
  mw::Twist v;

  orca_base::plan_pose_sync(cxt, p0, p1, p2, p3, a, v);

  std::cout << "p0: " << p0 << std::endl;
  std::cout << "p1: " << p1 << std::endl;
  std::cout << "p2: " << p2 << std::endl;
  std::cout << "p3: " << p3 << std::endl;
  std::cout << "a0: " << a << std::endl;
  std::cout << "v1: " << v << std::endl;

  // Make sure plan_sync didn't screw with the original pose
  assert(CLOSE_ENOUGH(p3_save.pose().position().x() - p3.pose().position().x()));
  assert(CLOSE_ENOUGH(p3_save.pose().position().y() - p3.pose().position().y()));
  assert(CLOSE_ENOUGH(p3_save.pose().position().z() - p3.pose().position().z()));
  assert(CLOSE_ENOUGH(orca::norm_angle(p3_save.pose().orientation().yaw() - p3.pose().orientation().yaw())));

  auto ramp = p1.pose() - p0.pose();
  auto run = p2.pose() - p1.pose();
  auto check = p0.pose() + ramp + run + ramp;

  // Make sure the sums work out
  assert(CLOSE_ENOUGH(p3_save.pose().position().x() - check.position().x()));
  assert(CLOSE_ENOUGH(p3_save.pose().position().y() - check.position().y()));
  assert(CLOSE_ENOUGH(p3_save.pose().position().z() - check.position().z()));
  assert(CLOSE_ENOUGH(orca::norm_angle(p3_save.pose().orientation().yaw() - check.orientation().yaw())));

  // TODO check against known results

  std::cout << "success" << std::endl;
}
