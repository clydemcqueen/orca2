#ifndef ORCA_SHARED_MW_EFFORTS_HPP
#define ORCA_SHARED_MW_EFFORTS_HPP

#include "orca_msgs/msg/efforts.hpp"
#include "orca_shared/model.hpp"
#include "orca_shared/mw/acceleration.hpp"
#include "orca_shared/util.hpp"

namespace mw
{

  class Efforts
  {
    orca_msgs::msg::Efforts msg_;

  public:

    Efforts() = default;

    explicit Efforts(const orca_msgs::msg::Efforts &msg)
    {
      msg_.forward = msg.forward;
      msg_.strafe = msg.strafe;
      msg_.vertical = msg.vertical;
      msg_.yaw = msg.yaw;
    }

    Efforts(const orca::Model &model, const double current_yaw, const Acceleration &u_bar)
    {
      // Convert from world frame to body frame
      double x_effort = model.accel_to_effort_xy(u_bar.x());
      double y_effort = model.accel_to_effort_xy(u_bar.y());
      double forward_effort, strafe_effort;
      orca::rotate_frame(x_effort, y_effort, current_yaw, forward_effort, strafe_effort);

      forward(forward_effort);
      strafe(strafe_effort);
      vertical(model.accel_to_effort_z(u_bar.z()));
      yaw(model.accel_to_effort_yaw(u_bar.yaw()));
    }

    orca_msgs::msg::Efforts to_msg() const
    {
      return msg_;
    }

    double forward() const
    {
      return msg_.forward;
    }

    double strafe() const
    {
      return msg_.strafe;
    }

    double vertical() const
    {
      return msg_.vertical;
    }

    double yaw() const
    {
      return msg_.yaw;
    }

    void forward(const double &v)
    {
      msg_.forward = orca::clamp(v, -1.0, 1.0);
    }

    void strafe(const double &v)
    {
      msg_.strafe = orca::clamp(v, -1.0, 1.0);
    }

    void vertical(const double &v)
    {
      msg_.vertical = orca::clamp(v, -1.0, 1.0);
    }

    void yaw(const double &v)
    {
      msg_.yaw = orca::clamp(v, -1.0, 1.0);
    }

    void all_stop()
    {
      msg_.forward = 0;
      msg_.strafe = 0;
      msg_.vertical = 0;
      msg_.yaw = 0;
    }

    Acceleration acceleration(const orca::Model &model, const double current_yaw) const
    {
      // Acceleration to effort
      double forward_accel = model.effort_to_accel_xy(msg_.forward);
      double strafe_accel = model.effort_to_accel_xy(msg_.strafe);
      double z_accel = model.effort_to_accel_z(msg_.vertical);
      double yaw_accel = model.effort_to_accel_yaw(msg_.yaw);

      // Rotate from body frame to world frame
      double x_accel, y_accel;
      orca::rotate_frame(forward_accel, strafe_accel, -current_yaw, x_accel, y_accel);

      return Acceleration{x_accel, y_accel, z_accel, yaw_accel};
    }
  };

  std::ostream &operator<<(std::ostream &os, Efforts const &e);

}

#endif //ORCA_SHARED_MW_EFFORTS_HPP
