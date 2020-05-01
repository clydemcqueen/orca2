#include "orca_description/parser.hpp"

#include "urdf/model.h"

namespace orca_description
{

  bool Parser::parse()
  {
    urdf::Model model;

    auto result = model.initFile(filename) &&
           get_joint(model, barometer_joint, t_baro_base) &&
           get_joint(model, forward_camera_joint, t_fcam_base) &&
           get_joint(model, left_camera_joint, t_lcam_base) &&
           get_joint(model, right_camera_joint, t_rcam_base);

    t_base_baro = t_baro_base.inverse();
    t_base_fcam = t_fcam_base.inverse();
    t_base_lcam = t_lcam_base.inverse();
    t_base_rcam = t_rcam_base.inverse();

    return result;
  }

  bool Parser::get_joint(const urdf::Model &model, const std::string &joint_name, tf2::Transform &t)
  {
    auto joint = model.getJoint(joint_name);

    if (!joint) {
      std::cout << "joint " << joint_name << " missing" << std::endl;
      return false;
    }

    if (joint->parent_link_name != base_link) {
      std::cout << "joint " << joint_name << " expected parent " << base_link
                << ", found " << joint->parent_link_name << std::endl;
      return false;
    }

    auto pose = joint->parent_to_joint_origin_transform;

    tf2::Transform t2 = tf2::Transform{
      tf2::Quaternion{pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w},
      tf2::Vector3{pose.position.x, pose.position.y, pose.position.z}};

    t = t2.inverse();

    return true;

  }

}