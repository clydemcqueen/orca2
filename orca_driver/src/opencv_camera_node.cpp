#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace opencv_camera {

// Target fps = 30 / (SKIP + 1)
const int SKIP = 0;

// Camera info is for the BlueROV2 low light USB camera
#ifdef RUN_INSIDE_CLION
std::string cfg_path("../cfg/camera_info.txt");
#else
std::string cfg_path("install/orca_driver/share/orca_driver/cfg/camera_info.txt");
#endif

// TODO camera_info_manager
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

class OpencvCameraNode: public rclcpp::Node
{
  cv::VideoCapture camera_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;
  std_msgs::msg::Header header_{};

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

public:

  explicit OpencvCameraNode(): Node("opencv_camera"), camera_(0)
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
  }

  ~OpencvCameraNode() {}

  void spin()
  {
    cv::Mat frame;

    while (rclcpp::ok()) {
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

      // Publish image_raw
      if (count_subscribers(image_pub_->get_topic_name()) > 0) {
        header_.stamp = stamp;
        cv_bridge::CvImage image{header_, sensor_msgs::image_encodings::BGR8, frame};
        image_pub_->publish(image.toImageMsg());
      }

      // Publish camera_info
      if (count_subscribers(camera_info_pub_->get_topic_name()) > 0) {
        camera_info_msg_.header.stamp = stamp;
        camera_info_pub_->publish(camera_info_msg_);
      }
    }
  }
};

} // namespace opencv_camera

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Create node
  auto node = std::make_shared<opencv_camera::OpencvCameraNode>();

  // Spin until rclcpp::ok() returns false
  node->spin();

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
