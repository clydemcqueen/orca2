#ifndef ORCA_SHARED_GEOMETRY_HPP
#define ORCA_SHARED_GEOMETRY_HPP

#include "orca_msgs/msg/efforts.hpp"
#include "orca_shared/model.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/time.hpp"

// 4 DoF geometry and motion structs

namespace orca
{

  //=====================================================================================
  // Pose in world frame
  //=====================================================================================

  struct Twist;
  struct Acceleration;

  struct Pose
  {
    double x{};
    double y{};
    double z{};
    double yaw{};

    constexpr Pose() = default;

    constexpr Pose(double _x, double _y, double _z, double _yaw) : x{_x}, y{_y}, z{_z}, yaw{_yaw}
    {}

    explicit Pose(const geometry_msgs::msg::Pose &msg);

    geometry_msgs::msg::Pose to_msg() const;

    // Distance between 2 poses on the xy plane
    double distance_xy(const Pose &that) const;

    double distance_xy(double _x, double _y) const;

    double distance_xy(const nav_msgs::msg::Odometry &msg) const;

    double distance_xy(const geometry_msgs::msg::Pose &msg) const;

    // Z distance between 2 poses
    double distance_z(const Pose &that) const;

    double distance_z(const nav_msgs::msg::Odometry &msg) const;

    // Yaw distance between 2 poses
    double distance_yaw(const Pose &that) const;

    double distance_yaw(const nav_msgs::msg::Odometry &msg) const;

    // Project pose at time t to time t+d, given initial velocity v0 and acceleration a
    Pose project(const rclcpp::Duration &d, const orca::Twist &v0, const orca::Acceleration &a) const;

    Pose operator+(const Pose &that) const;

    Pose operator-(const Pose &that) const;
  };

  std::ostream &operator<<(std::ostream &os, Pose const &pose);

  //=====================================================================================
  // PoseStamped
  //=====================================================================================

  struct PoseStamped
  {
    rclcpp::Time t{0, 0, RCL_ROS_TIME};
    Pose pose{};

    constexpr PoseStamped() = default;

    explicit PoseStamped(const geometry_msgs::msg::PoseStamped &msg) :
      t{msg.header.stamp},
      pose{msg.pose}
    {}

    explicit PoseStamped(const nav_msgs::msg::Odometry &msg) :
      t{msg.header.stamp},
      pose{msg.pose.pose}
    {}

    explicit PoseStamped(geometry_msgs::msg::PoseWithCovarianceStamped &msg) :
      t{msg.header.stamp},
      pose{msg.pose.pose}
    {}

    geometry_msgs::msg::PoseStamped to_msg() const;

    void add_to_path(nav_msgs::msg::Path &path) const;
  };

  std::ostream &operator<<(std::ostream &os, const orca::PoseStamped &p);

  //=====================================================================================
  // PoseWithCovariance
  //=====================================================================================

  struct PoseWithCovariance
  {
    Pose pose{};

    // Don't store the full 6x6 covariance, just store 4 bits of data
    bool x_valid{};
    bool y_valid{};
    bool z_valid{};
    bool yaw_valid{};

    constexpr PoseWithCovariance() = default;

    explicit PoseWithCovariance(const geometry_msgs::msg::PoseWithCovariance &pose_with_covariance);

    bool good_pose() const
    { return x_valid && y_valid && z_valid && yaw_valid; }

    bool good_z() const
    { return z_valid; }
  };

  //=====================================================================================
  // Twist in the world frame
  //=====================================================================================

  struct Twist
  {
    double x{};
    double y{};
    double z{};
    double yaw{};

    constexpr Twist() = default;

    constexpr Twist(double _x, double _y, double _z, double _yaw) : x{_x}, y{_y}, z{_z}, yaw{_yaw}
    {}

    explicit Twist(const geometry_msgs::msg::Twist &msg);

    geometry_msgs::msg::Twist to_msg() const;

    // Project twist at time t to time t+d, given acceleration a
    Twist project(const rclcpp::Duration &d, const orca::Acceleration &a) const;
  };

  std::ostream &operator<<(std::ostream &os, Twist const &t);

  //=====================================================================================
  // Twist in the body frame
  //=====================================================================================

  struct TwistBody
  {
    double forward{};
    double strafe{};
    double vertical{};
    double yaw{};

    constexpr TwistBody() = default;
  };

  //=====================================================================================
  // Acceleration in the world frame
  //=====================================================================================

  struct Acceleration
  {
    double x{};
    double y{};
    double z{};
    double yaw{};

    constexpr Acceleration() = default;

    constexpr Acceleration(double _x, double _y, double _z, double _yaw) : x{_x}, y{_y}, z{_z}, yaw{_yaw}
    {}

    Acceleration operator+(const Acceleration &that) const;

    Acceleration operator-(const Acceleration &that) const;

    Acceleration operator-() const;
  };

  std::ostream &operator<<(std::ostream &os, Acceleration const &a);

  //=====================================================================================
  // Acceleration in the body frame
  //=====================================================================================

  struct AccelerationBody
  {
    double forward{};
    double strafe{};
    double vertical{};
    double yaw{};

    constexpr AccelerationBody() = default;

    constexpr AccelerationBody(double _forward, double _strafe, double _vertical, double _yaw) :
      forward{_forward}, strafe{_strafe}, vertical{_vertical}, yaw{_yaw}
    {}
  };

  std::ostream &operator<<(std::ostream &os, AccelerationBody const &a);

  //=====================================================================================
  // Thruster efforts from joystick or pid controllers, in the body frame
  // Ranges from 1.0 for forward to -1.0 for reverse
  //=====================================================================================

  class Efforts
  {
    double forward_{};
    double strafe_{};
    double vertical_{};
    double yaw_{};

  public:

    constexpr Efforts() = default;

    Efforts(const orca::Model &model, const double current_yaw, const Acceleration &u_bar);

    explicit Efforts(const orca_msgs::msg::Efforts &msg);

    double forward() const
    { return forward_; }

    double strafe() const
    { return strafe_; }

    double vertical() const
    { return vertical_; }

    double yaw() const
    { return yaw_; }

    void set_forward(double forward);

    void set_strafe(double strafe);

    void set_vertical(double vertical);

    void set_yaw(double yaw);

    void all_stop();

    void to_acceleration(const orca::Model &model, const double current_yaw, Acceleration &u_bar);

    void scale(double factor);

    orca_msgs::msg::Efforts to_msg() const;
  };

  std::ostream &operator<<(std::ostream &os, Efforts const &e);

} // namespace orca_shared

#endif //ORCA_SHARED_GEOMETRY_HPP
