#ifndef ORCA_DRIVER_H
#define ORCA_DRIVER_H

#include <string>
#include <vector>

#include "mraa/common.hpp"
#include "mraa/led.hpp"
#include "br_ms5837/MS5837.h"

#include "rclcpp/rclcpp.hpp"

#include "orca_driver/driver_context.hpp"
#include "orca_driver/maestro.hpp"
#include "orca_msgs/msg/barometer.hpp"
#include "orca_msgs/msg/control.hpp"
#include "orca_msgs/msg/driver.hpp"

namespace orca_driver
{

  struct Thruster
  {
    int channel_;
    bool reverse_;
  };

  // DriverNode provides the interface between the Orca hardware and ROS.

  class DriverNode : public rclcpp::Node
  {
    // Parameters
    DriverContext cxt_;
    std::vector<Thruster> thrusters_;

    // State
    maestro::Maestro maestro_;
    orca_msgs::msg::Barometer barometer_msg_;
    orca_msgs::msg::Driver driver_msg_;

    // Control message state
    rclcpp::Subscription<orca_msgs::msg::Control>::SharedPtr control_sub_;
    rclcpp::Time control_msg_time_;

    // Timer
    rclcpp::TimerBase::SharedPtr spin_timer_;

    // Publications
    rclcpp::Publisher<orca_msgs::msg::Barometer>::SharedPtr barometer_pub_;
    rclcpp::Publisher<orca_msgs::msg::Driver>::SharedPtr driver_pub_;

    // LEDs on the UP board
    // https://github.com/intel-iot-devkit/mraa/blob/master/examples/platform/up2-leds.cpp
    mraa::Led led_ready_{"yellow"};
    mraa::Led led_mission_{"green"};
    mraa::Led led_problem_{"red"};

    // Barometer on i2c bus 0
    MS5837 barometer_;

    void validate_parameters();

    void set_status(uint8_t status);

    void control_callback(const orca_msgs::msg::Control::SharedPtr msg);

    void timer_callback();

    bool read_barometer();

    bool read_battery();

    bool read_leak();

    bool connect_controller();

    bool connect_barometer();

    bool connect_battery();

    bool connect_leak();

    void all_stop();

    void abort();

  public:
    explicit DriverNode();

    ~DriverNode()
    {}; // Suppress default copy and move constructors

    bool connect();

    void disconnect();
  };

} // namespace orca_driver

#endif // ORCA_DRIVER_H
