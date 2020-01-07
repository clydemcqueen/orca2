#include "fiducial_vlam/fiducial_vlam.hpp"
#include "gscam/gscam_node.hpp"

#include "rclcpp/rclcpp.hpp"

// Launch 2 nodes with use_intra_process_comms=true

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Create NodeOptions, turn on IPC
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  // Create GSCamNode
  auto gscam_node = std::make_shared<gscam::GSCamNode>(options);

  // Create VLocNode
  auto vloc_node = fiducial_vlam::vloc_node_factory(options);

  // Add both nodes to a single-threaded executor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(gscam_node);
  executor.add_node(vloc_node);

  // Spin until rclcpp::ok() returns false
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
