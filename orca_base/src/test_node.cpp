#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "orca_base/monotonic.hpp"

class TestNode : public rclcpp::Node
{
  orca_base::Monotonic<TestNode *, const sensor_msgs::msg::Joy::SharedPtr> cb_{this, &TestNode::process_joy};
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;

public:

  TestNode() : Node("test_node")
  {
    sub_ = create_subscription<sensor_msgs::msg::Joy>("joy",
      [this](const sensor_msgs::msg::Joy::SharedPtr msg) -> void { this->cb_.call(msg); });
  }

  void process_joy(const sensor_msgs::msg::Joy::SharedPtr msg, bool first)
  {
    RCLCPP_INFO(get_logger(), "joy %d", first);
  }
};

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Init node
  auto node = std::make_shared<TestNode>();

  // Spin node
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
