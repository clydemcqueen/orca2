#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"

#include "rclcpp/rclcpp.hpp"
#include "gazebo_ros/node.hpp"

#include "orca_base/pwm.hpp"
#include "orca_msgs/msg/control.hpp"

/* A simple thruster plugin. Usage:
 *
 *    <gazebo>
 *      <plugin name="OrcaThrusterPlugin" filename="libOrcaThrusterPlugin.so">
 *        <link_name>base_link</link_name>
 *        <ros_topic>/control</ros_topic>
 *        <thruster>
 *          <pos_force>50</pos_force>
 *          <neg_force>40</neg_force>
 *          <origin xyz="0.1 0.15 0" rpy="0 ${PI/2} ${PI*3/4}"/>
 *        </thruster>
 *      </plugin>
 *    </gazebo>
 *
 * We listen for Orca Control messages and apply thrust forces to base_link.
 *
 * There can the multiple <thruster> tags; the number and order of <thruster> tags must match
 * the number and order of int32s in the Control message. Each int32 indicates ESC pulse width and
 * ranges from 1100 (full reverse) through 1500 (stop) to 1900 (full forward).
 *
 *    <ros_topic> topic for Control messages. Default is /control.
 *    <pos_force> force (N) with max positive effort (pwm=1900). Default 50. Use negative if prop is reversed.
 *    <neg_force> force (N) with max negative effort (pwm=1100). Default is 40. Use negative if prop is reversed.
 *    <origin> thruster pose relative to base_link. Default is 0, 0, 0, 0, 0, 0.
 *
 * Note: the ROS URDF to SDF translation drops all fixed joints, collapsing all links into a single link.
 * There are several possible workarounds:
 * 1. Copy/paste the <origin> tags from each thruster <joint> tag to the <thruster> tag.
 * 2. Use non-fixed joints with motion limits.
 * 3. Use the <dontcollapsejoints> tag. (appears to require SDF 2.0)
 */

namespace gazebo {

// TODO(Crystal): use <ros> tags w/ parameters to simplify the parameter blocks for Orca plugins

constexpr double T200_MAX_POS_FORCE = 50;
constexpr double T200_MAX_NEG_FORCE = 40;

class OrcaThrusterPlugin : public ModelPlugin
{
  // Pointer to our base_link
  physics::LinkPtr base_link_;

  // Pointer to the Gazebo update event connection
  event::ConnectionPtr update_connection_;

  // Pointer to the GazeboROS node
  gazebo_ros::Node::SharedPtr node_;

  // ROS subscriber
  rclcpp::Subscription<orca_msgs::msg::Control>::SharedPtr thruster_sub_;

  // Model for each thruster
  struct Thruster
  {
    // Specified in the SDF, and doesn't change:
    ignition::math::Vector3d xyz;
    ignition::math::Vector3d rpy;
    double pos_force;
    double neg_force;

    // From the latest ROS message:
    double effort; // Range -1.0 to 1.0
  };

  // Our array of thrusters
  std::vector<Thruster> thrusters_ = {};

public:

  // Called once when the plugin is loaded.
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf)
  {
    // Get the GazeboROS node
    node_ = gazebo_ros::Node::Get(sdf);

    // Look for our link name
    std::string link_name = "base_link";
    if (sdf->HasElement("link_name"))
    {
      link_name = sdf->GetElement("link_name")->Get<std::string>();
    }
    RCLCPP_INFO(node_->get_logger(), "Thrust force will be applied to %s", link_name.c_str());
    base_link_ = model->GetLink(link_name);
    GZ_ASSERT(base_link_ != nullptr, "Missing link");

    // Look for our ROS topic
    std::string ros_topic = "/control";
    if (sdf->HasElement("ros_topic"))
    {
      ros_topic = sdf->GetElement("ros_topic")->Get<std::string>();
    }
    RCLCPP_INFO(node_->get_logger(), "Listening on %s", ros_topic.c_str());

    // Subscribe to the topic
    // Note the use of std::placeholders::_1 vs. the included _1 from Boost
    thruster_sub_ = node_->create_subscription<orca_msgs::msg::Control>(ros_topic, std::bind(&OrcaThrusterPlugin::OnRosMsg, this, std::placeholders::_1));
    
    // Listen to the update event. This event is broadcast every simulation iteration.
    update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&OrcaThrusterPlugin::OnUpdate, this, _1));
    
    // Look for <thruster> tags
    for (sdf::ElementPtr elem = sdf->GetElement("thruster"); elem; elem = elem->GetNextElement("thruster"))
    {
      Thruster t = {};
      t.pos_force = T200_MAX_POS_FORCE;
      t.neg_force = T200_MAX_NEG_FORCE;

      if (elem->HasElement("pos_force"))
      {
        t.pos_force = elem->GetElement("pos_force")->Get<double>();
      }

      if (elem->HasElement("neg_force"))
      {
        t.neg_force = elem->GetElement("neg_force")->Get<double>();
      }

      if (elem->HasElement("origin"))
      {
        sdf::ElementPtr origin = elem->GetElement("origin");
        if (origin->HasAttribute("xyz"))
        {
          origin->GetAttribute("xyz")->Get(t.xyz);
        }

        if (origin->HasAttribute("rpy"))
        {
          origin->GetAttribute("rpy")->Get(t.rpy);
        }
      }

      RCLCPP_INFO(node_->get_logger(), "Thruster pos %g neg %g xyz {%g, %g, %g} rpy {%g, %g, %g}",
        t.pos_force, t.neg_force, t.xyz.X(), t.xyz.Y(), t.xyz.Z(), t.rpy.X(), t.rpy.Y(), t.rpy.Z());
      thrusters_.push_back(t);
    }
  }

  // Handle an incoming message from ROS
  void OnRosMsg(const orca_msgs::msg::Control::SharedPtr msg)
  {
    for (int i = 0; i < thrusters_.size() && i < msg->thruster_pwm.size(); ++i)
    {
      thrusters_[i].effort = orca_base::pwm_to_effort(msg->thruster_pwm[i]);
    }
  }

  // Called by the world update start event, up to 1000 times per second.
  // TODO don't apply thrust force if we're above the surface of the water
  void OnUpdate(const common::UpdateInfo & /*info*/)
  {
    for (Thruster t : thrusters_)
    {
      // Default thruster force points directly up
      ignition::math::Vector3d force = {0.0, 0.0, t.effort * (t.effort < 0 ? t.neg_force : t.pos_force)};

      // Rotate force into place on the frame
      ignition::math::Quaternion<double> q{t.rpy};
      force = q.RotateVector(force);

      // Match base_link's current pose
      force = base_link_->WorldPose().Rot().RotateVector(force);

      // Apply the force to base_link
      base_link_->AddForceAtRelativePosition(force, t.xyz);
    }
  }
};

GZ_REGISTER_MODEL_PLUGIN(OrcaThrusterPlugin)

}
