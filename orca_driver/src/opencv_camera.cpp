#include "orca_driver/opencv_camera.hpp"

#include <iostream>
#include <fstream>

#include "cv_bridge/cv_bridge.h"

namespace opencv_camera {

// Target fps = 30 / (SKIP + 1)
const int SKIP = 0;

// Camera info is for the BlueROV2 low light USB camera
// TODO use camera_info_manager
#ifdef RUN_INSIDE_CLION
std::string cfg_path("../cfg/brusb_info.txt");
#else
std::string cfg_path("install/orca_driver/share/orca_driver/cfg/brusb_info.txt");
#endif

bool get_camera_info(sensor_msgs::msg::CameraInfo &info)
{
  // File format: 2 ints and 9 floats, separated by whitespace:
  // height width fx fy cx cy k1 k2 t1 t2 k3

  std::ifstream file;
  file.open(cfg_path);
  if (!file) {
    return false;
  }

  uint32_t height, width;
  double fx, fy, cx, cy, k1, k2, t1, t2, k3;
  file >> height >> width;
  file >> fx >> fy;
  file >> cx >> cy;
  file >> k1 >> k2 >> t1 >> t2 >> k3;
  file.close();

  // See https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/CameraInfo.msg

  info.header.frame_id = "camera_frame";
  info.height = height;
  info.width = width;
  info.distortion_model = "plumb_bob";

  info.d.push_back(k1);
  info.d.push_back(k2);
  info.d.push_back(t1);
  info.d.push_back(t2);
  info.d.push_back(k3);

  info.k[0] = fx;
  info.k[1] = 0;
  info.k[2] = cx;
  info.k[3] = 0;
  info.k[4] = fy;
  info.k[5] = cy;
  info.k[6] = 0;
  info.k[7] = 0;
  info.k[8] = 1;

  info.p[0] = fx;
  info.p[1] = 0;
  info.p[2] = cx;
  info.p[3] = 0;
  info.p[4] = 0;
  info.p[5] = fy;
  info.p[6] = cy;
  info.p[7] = 0;
  info.p[8] = 0;
  info.p[9] = 0;
  info.p[10] = 1;
  info.p[11] = 0;

  return true;
}

std::string mat_type2encoding(int mat_type)
{
  switch (mat_type) {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("unsupported encoding type");
  }
}

OpencvCameraNode::OpencvCameraNode():
  Node("opencv_camera", "", true),    // Set use_intra_process_comms = true
  camera_(0)                          // Attach to /dev/video0
{
  camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 1);
  image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", 1);

  if (!camera_.isOpened()) {
    RCLCPP_ERROR(get_logger(), "cannot open camera");
  }

  if (!get_camera_info(camera_info_msg_)) {
    RCLCPP_ERROR(get_logger(), "cannot get camera info");
  }

  header_.frame_id = "camera_frame";

  // Run loop on it's own thread
  thread_ = std::thread(std::bind(&OpencvCameraNode::loop, this));
}

OpencvCameraNode::~OpencvCameraNode()
{
  // Stop loop
  canceled_.store(true);
  if (thread_.joinable()) {
    thread_.join();
  }
}

void OpencvCameraNode::loop()
{
  cv::Mat frame;

  while (rclcpp::ok() && !canceled_.load()) {
    // Block until a frame is available
    camera_.read(frame);

    // Skip some frames while debugging to slow down the pipeline
    static int skip_count = 0;
    if (++skip_count < SKIP) {
      continue;
    }
    skip_count = 0;

    // Synchronize messages
    auto stamp = now();

    // Pass unique_ptr to publish() calls, these will not be copied
    sensor_msgs::msg::CameraInfo::UniquePtr camera_info_msg(new sensor_msgs::msg::CameraInfo(camera_info_msg_));
    sensor_msgs::msg::Image::UniquePtr image_msg(new sensor_msgs::msg::Image());

    // Convert OpenCV Mat to ROS Image
    image_msg->header.stamp = stamp;
    image_msg->header.frame_id = "camera_frame";
    image_msg->height = frame.rows;
    image_msg->width = frame.cols;
    image_msg->encoding = mat_type2encoding(frame.type());
    image_msg->is_bigendian = false;
    image_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    image_msg->data.assign(frame.datastart, frame.dataend);

    // Publish
    image_pub_->publish(image_msg);
    camera_info_pub_->publish(camera_info_msg_);
  }
}

} // namespace opencv_camera