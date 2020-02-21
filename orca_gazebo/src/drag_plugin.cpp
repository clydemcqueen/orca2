#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"

#include "orca_shared/model.hpp"

/* A simple drag plugin. Usage:
 *
 *    <gazebo>
 *      <plugin name="OrcaDragPlugin" filename="libOrcaDragPlugin.so">
 *        <link name="base_link">
 *          <fluid_density>997</fluid_density>
 *          <center_of_mass>0 0 -0.2</center_of_mass>
 *          <tether_attach>-0.5, -0.4, 0</tether_attach>
 *          <linear_drag>10 20 30</linear_drag>
 *          <angular_drag>5 10 15</angular_drag>
 *          <tether_drag>4</tether_drag>
 *        </link>
 *      </plugin>
 *    </gazebo>
 *
 *    <fluid_density> Fluid density in kg/m^3. Default is 997 (freshwater), use 1029 for seawater.
 *    <center_of_mass> Drag force is applied to the center of mass.
 *    <tether_attach> Relative position of tether attachment.
 *    <linear_drag> Linear drag constants. See default calculation.
 *    <angular_drag> Angular drag constants. See default calculation.
 *    <tether_drag> Tether drag constant. See default calculation.
 *
 * Limitations:
 *    Tether drag is modeled only in x
 */

namespace gazebo
{

  constexpr double FRESHWATER_DENSITY = 997;

  class OrcaDragPlugin : public ModelPlugin
  {
    physics::LinkPtr base_link_;

    // Drag force will be applied to the center_of_mass_ (body frame)
    ignition::math::Vector3d center_of_mass_{0, 0, 0};

    // Tether drag will be applied to the tether attachment point (body frame)
    ignition::math::Vector3d tether_attach_{0, 0, 0};

    // Drag constants (body frame)
    ignition::math::Vector3d linear_drag_;
    ignition::math::Vector3d angular_drag_;
    double tether_drag_;

    event::ConnectionPtr update_connection_;

  public:

    // Called once when the plugin is loaded.
    void Load(physics::ModelPtr model, sdf::ElementPtr sdf)
    {
      std::string link_name{"base_link"};
      double fluid_density = FRESHWATER_DENSITY;

      std::cout << std::endl;
      std::cout << "ORCA DRAG PLUGIN PARAMETERS" << std::endl;
      std::cout << "-----------------------------------------" << std::endl;
      std::cout << "Default link name: " << link_name << std::endl;
      std::cout << "Default fluid density: " << fluid_density << std::endl;
      std::cout << "Default center of mass: " << center_of_mass_ << std::endl;
      std::cout << "Default tether attachment point: " << tether_attach_ << std::endl;
      std::cout << "Default linear drag: " << linear_drag_ << std::endl;
      std::cout << "Default angular drag: " << angular_drag_ << std::endl;
      std::cout << "Default tether drag: " << tether_drag_ << std::endl;

      GZ_ASSERT(model != nullptr, "Model is null");
      GZ_ASSERT(sdf != nullptr, "SDF is null");

      if (sdf->HasElement("link")) {
        sdf::ElementPtr linkElem = sdf->GetElement("link"); // Only one link is supported

        if (linkElem->HasAttribute("name")) {
          linkElem->GetAttribute("name")->Get(link_name);
          std::cout << "Link name: " << link_name << std::endl;
        }

        if (linkElem->HasElement("fluid_density")) {
          fluid_density = linkElem->GetElement("fluid_density")->Get<double>();
          std::cout << "Fluid density: " << fluid_density << std::endl;
        }

        if (linkElem->HasElement("center_of_mass")) {
          center_of_mass_ = linkElem->GetElement("center_of_mass")->Get<ignition::math::Vector3d>();
          std::cout << "Center of mass: " << center_of_mass_ << std::endl;
        }

        if (linkElem->HasElement("tether_attach")) {
          tether_attach_ = linkElem->GetElement("tether_attach")->Get<ignition::math::Vector3d>();
          std::cout << "Tether attachment point: " << tether_attach_ << std::endl;
        }

        if (linkElem->HasElement("linear_drag")) {
          linear_drag_ = linkElem->GetElement("linear_drag")->Get<ignition::math::Vector3d>();
          std::cout << "Linear drag: " << linear_drag_ << std::endl;
        }

        if (linkElem->HasElement("angular_drag")) {
          angular_drag_ = linkElem->GetElement("angular_drag")->Get<ignition::math::Vector3d>();
          std::cout << "Angular drag: " << angular_drag_ << std::endl;
        }

        if (linkElem->HasElement("tether_drag")) {
          tether_drag_ = linkElem->GetElement("tether_drag")->Get<double>();
          std::cout << "Tether drag: " << tether_drag_ << std::endl;
        }
      }

      // Initialize Orca model from parameters
      orca::Model orca_;
      orca_.fluid_density_ = fluid_density;

      // Get drag constants
      // Angular drag is a wild guess, but should be non-zero
      linear_drag_ = {orca_.drag_const_f(), orca_.drag_const_s(), orca_.drag_const_z()};
      angular_drag_ = {orca_.drag_const_yaw(), orca_.drag_const_yaw(), orca_.drag_const_yaw()};
      tether_drag_ = orca_.tether_drag_const();

      base_link_ = model->GetLink(link_name);
      GZ_ASSERT(base_link_ != nullptr, "Missing link");

      // Listen for the update event. This event is broadcast every simulation iteration.
      update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&OrcaDragPlugin::OnUpdate, this, _1));

      std::cout << "-----------------------------------------" << std::endl;
      std::cout << std::endl;
    }

    // Called by the world update start event, up to 1000 times per second.
    void OnUpdate(const common::UpdateInfo & /*info*/)
    {
      ignition::math::Vector3d linear_velocity = base_link_->RelativeLinearVel();
      ignition::math::Vector3d angular_velocity = base_link_->RelativeAngularVel();

      ignition::math::Vector3d drag_force;
      drag_force.X() = linear_velocity.X() * fabs(linear_velocity.X()) * -linear_drag_.X();
      drag_force.Y() = linear_velocity.Y() * fabs(linear_velocity.Y()) * -linear_drag_.Y();
      drag_force.Z() = linear_velocity.Z() * fabs(linear_velocity.Z()) * -linear_drag_.Z();
      base_link_->AddLinkForce(drag_force, center_of_mass_);

      ignition::math::Vector3d drag_torque;
      drag_torque.X() = angular_velocity.X() * fabs(angular_velocity.X()) * -angular_drag_.X();
      drag_torque.Y() = angular_velocity.Y() * fabs(angular_velocity.Y()) * -angular_drag_.Y();
      drag_torque.Z() = angular_velocity.Z() * fabs(angular_velocity.Z()) * -angular_drag_.Z();
      base_link_->AddRelativeTorque(drag_torque); // ODE adds torque at the center of mass

      // Tether drag only accounts for motion in x (forward/reverse)
      ignition::math::Vector3d tether_force;
      double depth = -base_link_->WorldPose().Pos().Z();
      tether_force.X() = linear_velocity.X() * fabs(linear_velocity.X()) * depth * -tether_drag_;
      tether_force.Y() = 0;
      tether_force.Z() = 0;
      base_link_->AddLinkForce(tether_force, tether_attach_);
    }
  };

  GZ_REGISTER_MODEL_PLUGIN(OrcaDragPlugin)

}
