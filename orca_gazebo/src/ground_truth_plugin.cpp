#include "gazebo/gazebo.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"

#include "rclcpp/rclcpp.hpp"
#include "gazebo_ros/node.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "orca_gazebo/orca_gazebo_util.hpp"

/* Publish a ground truth pose for a link.
 * Usage:
 *
 *    <gazebo>
 *      <plugin name="OrcaGroundTruthPlugin" filename="libOrcaGroundTruthPlugin.so">
 *        <link name="base_link">
 *          <odom_topic>/ground_truth</odom_topic>
 *          <surface>10</surface>
 *        </link>
 *      </plugin>
 *    </gazebo>
 *
 *    <odom_topic> Topic for nav_msgs/Odometry messages. Default is /ground_truth.
 *    <surface> How far above z=0 the surface of the water is; used to calculate depth. Default is 10.
 */

namespace gazebo {

// TODO(Crystal): use <ros> tags w/ parameters to simplify the parameter blocks for Orca plugins
// TODO: do we need a tf broadcaster?

class OrcaGroundTruthPlugin : public ModelPlugin
{
  physics::LinkPtr base_link_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_pub_;
  event::ConnectionPtr update_connection_;

  double surface_ = 10;                       // Altitude of water surface above the ground
  gazebo::common::Time last_update_time_;     // Last time we got an update event
  double update_period_;                      // Seconds between GPS readings

public:

  // Called once when the plugin is loaded.
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf)
  {
    GZ_ASSERT(model != nullptr, "Model is null");
    GZ_ASSERT(sdf != nullptr, "SDF is null");

    // Get the GazeboROS node
    node_ = gazebo_ros::Node::Get(sdf);

    std::string link_name = "base_link";
    std::string odom_topic = "/ground_truth";

    std::cout << std::endl;
    std::cout << "ORCA GROUND TRUTH PLUGIN PARAMETERS" << std::endl;
    std::cout << "-----------------------------------------" << std::endl;
    std::cout << "Default link name: " << link_name << std::endl;
    std::cout << "Default odom topic: " << odom_topic << std::endl;
    std::cout << "Default surface: " << surface_ << std::endl;

    if (sdf->HasElement("link"))
    {
      sdf::ElementPtr linkElem = sdf->GetElement("link"); // Only one link is supported

      if (linkElem->HasAttribute("name"))
      {
        linkElem->GetAttribute("name")->Get(link_name);
        std::cout << "Link name: " << link_name << std::endl;
      }
    }

    if (sdf->HasElement("odom_topic"))
    {
      odom_topic = sdf->GetElement("odom_topic")->Get<std::string>(); // TODO why isn't this getting picked up?
      std::cout << "Odom topic: " << odom_topic << std::endl;
    }

    if (sdf->HasElement("surface"))
    {
      surface_ = sdf->GetElement("surface")->Get<double>();
      std::cout << "Surface: " << surface_ << std::endl;
    }

    base_link_ = model->GetLink(link_name);
    GZ_ASSERT(base_link_ != nullptr, "Missing link");

    // Set up ROS publisher
    ground_truth_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 1);

    // Listen for the update event. This event is broadcast every simulation iteration.
    update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&OrcaGroundTruthPlugin::OnUpdate, this, _1));

    // Start the timer
    update_period_ = 1 / 100;
    last_update_time_ = model->GetWorld()->SimTime();

    std::cout << "-----------------------------------------" << std::endl;
    std::cout << std::endl;
  }

  // Called by the world update start event, up to 1kHz
  void OnUpdate(const common::UpdateInfo& info)
  {
    gazebo::common::Time current_time = info.simTime;

    // Check for negative elapsed time, e.g., if the world is reset
    if (current_time < last_update_time_)
    {
      RCLCPP_INFO(node_->get_logger(), "Negative elapsed sim time");
      last_update_time_ = current_time;
    }

    // Check period
    if ((current_time - last_update_time_).Double() < update_period_)
    {
      return;
    }

    // Pose in world frame
    ignition::math::Pose3d pose = base_link_->WorldPose();

    // Linear velo in world frame
    ignition::math::Vector3d linear_vel = base_link_->WorldLinearVel();

    // TODO get angular velo in odom frame

    // TODO set covar (very small, and fixed)

    nav_msgs::msg::Odometry msg;
    msg.header.frame_id = "odom";
    msg.header.stamp = node_->now();
    msg.child_frame_id = "base_link";
    msg.pose.pose.position.x = pose.Pos().X();
    msg.pose.pose.position.y = pose.Pos().Y();;
    msg.pose.pose.position.z = pose.Pos().Z() - surface_;
    msg.pose.pose.orientation.x = pose.Rot().X();
    msg.pose.pose.orientation.y = pose.Rot().Y();
    msg.pose.pose.orientation.z = pose.Rot().Z();
    msg.pose.pose.orientation.w = pose.Rot().W();
    msg.twist.twist.linear.x = linear_vel.X();
    msg.twist.twist.linear.y = linear_vel.Y();
    msg.twist.twist.linear.z = linear_vel.Z();

    ground_truth_pub_->publish(msg);
  }
};

GZ_REGISTER_MODEL_PLUGIN(OrcaGroundTruthPlugin)

} // namespace gazebo
