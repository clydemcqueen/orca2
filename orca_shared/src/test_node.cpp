#include "orca_shared/fp.hpp"
#include "orca_shared/monotonic.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

class TestNode : public rclcpp::Node
{
  monotonic::Monotonic<TestNode *, const geometry_msgs::msg::PoseStamped::SharedPtr> cb_{this, &TestNode::process_pose};
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;

public:

  TestNode() : Node("test_node")
  {
    sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void
      { this->cb_.call(msg); });
  }

  void process_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg, bool first)
  {
    RCLCPP_INFO(get_logger(), "joy %d", first);
  }
};

void test_geometry_and_fp()
{
  orca::Pose pose;
  orca::PoseStamped pose_stamped;
  orca::PoseWithCovariance pose_with_covariance;
  orca::Twist twist;
  orca::TwistBody twist_body;
  orca::Acceleration acceleration;
  orca::AccelerationBody acceleration_body;
  orca::Efforts efforts;
  orca::Observation observation;
  orca::ObservationStamped observation_stamped;
  orca::FP fp;
  orca::FPStamped fp_stamped;
}

int main(int argc, char **argv)
{
  test_geometry_and_fp();

  // Force flush of the stdout buffer
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

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
