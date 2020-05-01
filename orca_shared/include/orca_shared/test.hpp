#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

sensor_msgs::msg::CameraInfo make_camera_info();
geometry_msgs::msg::Pose make_cam_f_base();

bool test_mw_pose_segment();
bool test_mw_roundtrip();
bool test_mw_observer();
bool test_mw_header();
bool test_mw_move();