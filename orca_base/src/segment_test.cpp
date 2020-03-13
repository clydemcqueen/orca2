#include "orca_base/pose_segment.hpp"

#include <iostream>

#include "orca_shared/util.hpp"

#define CLOSE_ENOUGH(x) (abs(x) < 0.001)

void segment_test()
{
  std::cout << "=== TRAP2 SEGMENT TEST ===" << std::endl;

  orca::PoseStamped p0, p1, p2, p3;

  p3.pose.x = -5;
  p3.pose.y = 0;
  p3.pose.z = 1;
  p3.pose.yaw = 2;

  auto p3_save = p3;

  orca_base::AUVContext cxt{};

  orca::Acceleration a;
  orca::Twist v;

  orca_base::plan_pose_sync(cxt, p0, p1, p2, p3, a, v);

  std::cout << "p0: " << p0 << std::endl;
  std::cout << "p1: " << p1 << std::endl;
  std::cout << "p2: " << p2 << std::endl;
  std::cout << "p3: " << p3 << std::endl;
  std::cout << "a0: " << a << std::endl;
  std::cout << "v1: " << v << std::endl;

  // Make sure plan_sync didn't screw with the original pose
  assert(CLOSE_ENOUGH(p3_save.pose.x - p3.pose.x));
  assert(CLOSE_ENOUGH(p3_save.pose.y - p3.pose.y));
  assert(CLOSE_ENOUGH(p3_save.pose.z - p3.pose.z));
  assert(CLOSE_ENOUGH(orca::norm_angle(p3_save.pose.yaw - p3.pose.yaw)));

  auto ramp = p1.pose - p0.pose;
  auto run = p2.pose - p1.pose;
  auto check = p0.pose + ramp + run + ramp;

  // Make sure the sums work out
  assert(CLOSE_ENOUGH(p3_save.pose.x - check.x));
  assert(CLOSE_ENOUGH(p3_save.pose.y - check.y));
  assert(CLOSE_ENOUGH(p3_save.pose.z - check.z));
  assert(CLOSE_ENOUGH(orca::norm_angle(p3_save.pose.yaw - check.yaw)));

  // TODO check against known results

  std::cout << "success" << std::endl;
}
