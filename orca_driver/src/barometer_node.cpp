#include "br_ms5837/MS5837.h"
#include "orca_msgs/msg/barometer.hpp"
#include "rclcpp/rclcpp.hpp"

namespace orca_driver
{

  constexpr int QUEUE_SIZE = 10;

  class BarometerNode : public rclcpp::Node
  {
    MS5837 barometer_;
    std::thread sensor_thread_;
    std::atomic<bool> stop_signal_;
    rclcpp::Publisher<orca_msgs::msg::Barometer>::SharedPtr barometer_pub_;

  public:

    BarometerNode() :
      Node{"barometer_node"},
      barometer_{0}
    {
      barometer_pub_ = create_publisher<orca_msgs::msg::Barometer>("barometer", QUEUE_SIZE);

      sensor_thread_ = std::thread(
        [this]()
        {
          RCLCPP_INFO(get_logger(), "sensor thread running");

          while (!stop_signal_ && rclcpp::ok()) {
            orca_msgs::msg::Barometer barometer_msg_;
            barometer_.read(); // Takes 40ms+
            barometer_msg_.header.stamp = now();
            barometer_msg_.pressure = barometer_.pressure() * 100; // Pascals
            barometer_msg_.temperature = barometer_.temperature(); // Celsius
            barometer_pub_->publish(barometer_msg_);
          }

          stop_signal_ = false;
          RCLCPP_INFO(get_logger(), "sensor thread stopped");
        });

      RCLCPP_INFO(get_logger(), "barometer_node running");
    }

    ~BarometerNode()
    {
      stop_signal_ = true;
      sensor_thread_.join();
    }
  };

} // namespace orca_driver

int main(int argc, char **argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<orca_driver::BarometerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
