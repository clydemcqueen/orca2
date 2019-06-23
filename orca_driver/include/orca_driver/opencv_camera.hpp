#ifndef ORCA_DRIVER_OPENCV_CAMERA_HPP
#define ORCA_DRIVER_OPENCV_CAMERA_HPP

#include "rclcpp/rclcpp.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace opencv_camera
{

  class OpencvCameraNode : public rclcpp::Node
  {
    std::thread thread_;
    std::atomic<bool> canceled_;

    cv::VideoCapture camera_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;
    std_msgs::msg::Header header_{};

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  public:

    explicit OpencvCameraNode();

    ~OpencvCameraNode();

  private:

    void loop();
  };

} // namespace opencv_camera

#endif //ORCA_DRIVER_OPENCV_CAMERA_HPP
