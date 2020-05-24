#include "rclcpp/rclcpp.hpp"

#include "orca_msgs/msg/control.hpp"

namespace orca_driver
{
  //=============================================================================
  // TestNode
  // Test Orca hardware by sending control messages
  //=============================================================================

  class TestNode : public rclcpp::Node
  {
    const int THRUST_DIFF = 50;

    rclcpp::TimerBase::SharedPtr spin_timer_;
    rclcpp::Publisher<orca_msgs::msg::Control>::SharedPtr control_pub_;

    void spin_once()
    {
      orca_msgs::msg::Control msg;
      msg.header.stamp = now();
      msg.odom_lag = 0.0;
      msg.mode = orca_msgs::msg::Control::ROV;
      msg.camera_tilt_pwm = orca_msgs::msg::Control::TILT_0;
      msg.brightness_pwm = orca_msgs::msg::Control::LIGHTS_OFF;

      // Rotate through all 6 thrusters, and send fwd and rev signals
      // Each thruster gets 5s, so a cycle is 30s
      int cycle = msg.header.stamp.sec % 30;
      int thruster = cycle / 5;
      static int prev_thruster = -1;
      if (thruster != prev_thruster) {
        RCLCPP_INFO(get_logger(), "test thruster %d, fwd, rev, stop", thruster + 1);
        prev_thruster = thruster;
      }
      int pwm;
      switch (cycle % 5) {
        case 1:
          pwm = orca_msgs::msg::Control::THRUST_STOP + THRUST_DIFF;
          break;
        case 3:
          pwm = orca_msgs::msg::Control::THRUST_STOP - THRUST_DIFF;
          break;
        default:
          pwm = orca_msgs::msg::Control::THRUST_STOP;
          break;
      }

      for (int i = 0; i < 6; ++i) {
        msg.thruster_pwm.push_back(i == thruster ? pwm : orca_msgs::msg::Control::THRUST_STOP);
      }

      control_pub_->publish(msg);
    }

  public:
    explicit TestNode() :
      Node{"self_test"}
    {
      // Suppress IDE warning
      (void) spin_timer_;

      // Publish control messages
      control_pub_ = create_publisher<orca_msgs::msg::Control>("control", 1);

      // Spin timer
      using namespace std::chrono_literals;
      spin_timer_ = create_wall_timer(100ms, std::bind(&TestNode::spin_once, this));
    }

    ~TestNode() override
    {
    }
  };

} // namespace orca_driver

//=============================================================================
// Main
//=============================================================================

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Init node
  auto node = std::make_shared<orca_driver::TestNode>();

  // Spin
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
