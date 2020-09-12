// Copyright (c) 2020, Clyde McQueen.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <memory>

#include "br_ms5837/MS5837.h"
#include "orca_msgs/msg/barometer.hpp"
#include "rclcpp/rclcpp.hpp"

namespace orca_driver
{

constexpr int QUEUE_SIZE = 10;
constexpr double BARO_VARIANCE = 201.7 * 201.7;      // Measured during ft3

class BarometerNode : public rclcpp::Node
{
#ifdef PROCESSOR_X86_64
  // UP board
  MS5837 barometer_{0};
#else
  // Raspberry Pi
  MS5837 barometer_{1, true};
#endif
  std::thread sensor_thread_;
  std::atomic<bool> stop_signal_{};
  rclcpp::Publisher<orca_msgs::msg::Barometer>::SharedPtr barometer_pub_;

public:
  BarometerNode()
  : Node{"barometer_node"}
  {
    barometer_pub_ = create_publisher<orca_msgs::msg::Barometer>("barometer", QUEUE_SIZE);

    sensor_thread_ = std::thread(
      [this]()
      {
        // TODO(clyde): init() returns true even when the barometer isn't attached, fix test
        if (!barometer_.init()) {
          RCLCPP_ERROR(get_logger(), "can't connect to barometer, correct bus? member of i2c?");
        } else {
          RCLCPP_INFO(get_logger(), "sensor thread running");

          while (!stop_signal_ && rclcpp::ok()) {
            orca_msgs::msg::Barometer barometer_msg;
            barometer_.read();    // Takes 40ms+
            barometer_msg.header.stamp = now();
            barometer_msg.pressure = barometer_.pressure() * 100;    // Pascals
            barometer_msg.pressure_variance = BARO_VARIANCE;         // Pascals
            barometer_msg.temperature = barometer_.temperature();    // Celsius
            barometer_pub_->publish(barometer_msg);
          }
        }

        RCLCPP_INFO(get_logger(), "sensor thread stopped");
      });

    RCLCPP_INFO(get_logger(), "barometer_node running");
  }

  ~BarometerNode() override
  {
    stop_signal_ = true;
    sensor_thread_.join();
  }
};

}  // namespace orca_driver

int main(int argc, char ** argv)
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<orca_driver::BarometerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
