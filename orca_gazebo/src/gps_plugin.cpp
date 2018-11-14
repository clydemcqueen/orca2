#include "gazebo/gazebo.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"

#include "rclcpp/rclcpp.hpp"
#include "gazebo_ros/node.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "orca_gazebo/orca_gazebo_util.hpp"

/* Simulate a GPS sensor attached to a mast on a submersible vehicle. Generates geometry_msgs/Vector3Stamped messages.
 * Usage:
 *
 *    <gazebo>
 *      <plugin name="OrcaGPSPlugin" filename="libOrcaGPSPlugin.so">
 *        <link name="base_link">
 *          <pose_topic>/gps</pose_topic>
 *          <mast_height>0.5</mast_height>
 *          <surface>10</surface>
 *        </link>
 *      </plugin>
 *    </gazebo>
 *
 *    <pose_topic> Topic for geometry_msgs/PoseWithCovarianceStamped messages. Default is /gps.
 *    <mast_height> Height of the antenna mast above the water. Default is 0.5.
 *    <surface> How far above z=0 the surface of the water is; used to calculate depth.
 */

namespace gazebo {

// TODO(Crystal): use <ros> tags w/ parameters to simplify the parameter blocks for Orca plugins
// TODO do we need a tf broadcaster?

// https://www.gps.gov/systems/gps/performance/accuracy/
constexpr double GPS_STDDEV = 1.891 / 2;

class OrcaGPSPlugin : public ModelPlugin
{
  physics::LinkPtr base_link_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped >::SharedPtr gps_pub_;
  event::ConnectionPtr update_connection_;

  double time_above_surface_ = 0;             // Seconds above the surface
  double mast_height_ = 0.50;                 // Height of antenna mast
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
    std::string pose_topic = "/gps";

    std::cout << std::endl;
    std::cout << "ORCA GPS PLUGIN PARAMETERS" << std::endl;
    std::cout << "-----------------------------------------" << std::endl;
    std::cout << "Default link name: " << link_name << std::endl;
    std::cout << "Default pose topic: " << pose_topic << std::endl;
    std::cout << "Default mast height: " << mast_height_ << std::endl;
    std::cout << "Default surface: " << surface_ << std::endl;

    if (sdf->HasElement("link"))
    {
      sdf::ElementPtr linkElem = sdf->GetElement("link"); // Only one link is supported

      if (linkElem->HasAttribute("name"))
      {
        linkElem->GetAttribute("name")->Get(link_name);
        std::cout << "Link name: " << link_name << std::endl;
      }

      if (linkElem->HasElement("pose_topic")) // TODO should be child of gazebo element, not link element
      {
        pose_topic = linkElem->GetElement("pose_topic")->Get<std::string>();
        std::cout << "Pose topic: " << pose_topic << std::endl;
      }

      if (linkElem->HasElement("mast_height"))
      {
        mast_height_ = linkElem->GetElement("mast_height")->Get<double>();
        std::cout << "Mast height: " << mast_height_ << std::endl;
      }

      if (linkElem->HasElement("surface")) // TODO should be child of gazebo element, not link element
      {
        surface_ = linkElem->GetElement("surface")->Get<double>();
        std::cout << "Surface: " << surface_ << std::endl;
      }
    }

    base_link_ = model->GetLink(link_name);
    GZ_ASSERT(base_link_ != nullptr, "Missing link");

    // Set up ROS publisher
    gps_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic, 1);

    // Listen for the update event. This event is broadcast every simulation iteration.
    update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&OrcaGPSPlugin::OnUpdate, this, _1));

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

    // Is the GPS sensor above the water surface?
    ignition::math::Vector3d pos = base_link_->WorldPose().Pos();
    if (pos.Z() + mast_height_ > surface_)
    {
      // Do we have a satellite fix? Assume it takes 5 seconds
      if (time_above_surface_ > 5)
      {
        // Publish pose
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.frame_id = "odom";
        msg.header.stamp = node_->now();
        msg.pose.pose.position.x = orca_gazebo::gaussianKernel(pos.X(), GPS_STDDEV);
        msg.pose.pose.position.y = orca_gazebo::gaussianKernel(pos.Y(), GPS_STDDEV);
        msg.pose.pose.position.z = orca_gazebo::gaussianKernel(pos.Z() - surface_, GPS_STDDEV);
        msg.pose.covariance[0] = GPS_STDDEV * GPS_STDDEV;
        msg.pose.covariance[7] = GPS_STDDEV * GPS_STDDEV;
        gps_pub_->publish(msg);
      }
      else
      {
        time_above_surface_ += (current_time - last_update_time_).Double();
      }
    }
    else
    {
      time_above_surface_ = 0;
    }
  }
};

GZ_REGISTER_MODEL_PLUGIN(OrcaGPSPlugin)

} // namespace gazebo
