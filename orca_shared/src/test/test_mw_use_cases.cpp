#include "orca_shared/mw/mw.hpp"
#include "orca_shared/test.hpp"

bool test_map()
{
  std::cout << "=== TEST MAP ===" << std::endl;

  // TODO

  std::cout << "success" << std::endl;
  return true;
}

bool test_mw_observer()
{
  std::cout << "=== TEST OBSERVER ===" << std::endl;

  image_geometry::PinholeCameraModel cam_model;
  cam_model.fromCameraInfo(make_camera_info());

  tf2::Transform t_base_cam;
  tf2::fromMsg(make_cam_f_base(), t_base_cam);

  const mw::Observer observer{0.2, cam_model, t_base_cam};
  std::cout << observer << std::endl;

  const mw::PolarObservation po1{1, 10, 0};
  const mw::PolarObservation po2{2, 10, 0.5};

  mw::Observations observations{observer};
  observations.add_polar(po1);
  observations.add_polar(po2);
  std::cout << observations << std::endl;

  std::cout << "success" << std::endl;
  return true;
}

bool test_mw_pose_segment()
{
  std::cout << "=== TEST POSE SEGMENT ===" << std::endl;

  mw::Header header{rclcpp::Time{0, 999, RCL_ROS_TIME}, "map"};
  mw::Pose pose{0, 0, 0, 0};
  mw::PoseStamped plan{header, pose};

  mw::Pose goal{plan.pose()};
  goal.position().x() = 5;
  goal.position().y() = 10;
  goal.orientation().yaw(0.5);
  std::cout << goal << std::endl;

  const int num_iter = 10;
  auto xy_distance = goal.position().distance_xy(plan.pose().position());
  auto yaw_distance = goal.orientation().distance_yaw(plan.pose().orientation());
  auto angle_to_goal = std::atan2(goal.position().y() - plan.pose().position().y(),
                                  goal.position().x() - plan.pose().position().x());

  for (int i = 0; i < num_iter; ++i) {
    plan.pose().position().x() += std::cos(angle_to_goal) * xy_distance / num_iter;
    plan.pose().position().y() += std::sin(angle_to_goal) * xy_distance / num_iter;
    plan.pose().orientation().yaw(plan.pose().orientation().yaw() + yaw_distance / num_iter);
    std::cout << plan.pose() << std::endl;
  }

  std::cout << "success" << std::endl;
  return true;
}

bool test_mw_polar_segment()
{
  std::cout << "=== TEST POLAR SEGMENT ===" << std::endl;

  // TODO

  std::cout << "success" << std::endl;
  return true;
}
