#include <iostream>
#include "orca_shared/test.hpp"

sensor_msgs::msg::CameraInfo make_camera_info()
{
  sensor_msgs::msg::CameraInfo camera_info;
  camera_info.width = 800;
  camera_info.height = 600;
  camera_info.distortion_model = "plumb_bob";

  camera_info.k[0] = camera_info.p[0] = 474.89673285067175;
  camera_info.k[2] = camera_info.p[2] = 400.5;
  camera_info.k[4] = camera_info.p[4] = 474.89673285067175;
  camera_info.k[5] = camera_info.p[5] = 300.5;
  camera_info.k[8] = camera_info.p[8] = 1;

  camera_info.r[0] = 1;
  camera_info.r[4] = 1;
  camera_info.r[8] = 1;

  return camera_info;
}

geometry_msgs::msg::Pose make_cam_f_base()
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1;
  pose.orientation.x = 0.5;
  pose.orientation.y = -0.5;
  pose.orientation.z = 0.5;
  pose.orientation.w = -0.5;
  return pose;
}

int main(int argc, char **argv)
{
  if (test_mw_move() &&
  test_mw_header() &&
      test_mw_observer() &&
      test_mw_roundtrip() &&
      test_mw_pose_segment()) {
    std::cout << "all successful" << std::endl;
  }
}