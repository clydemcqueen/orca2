#ifndef ORCA_SHARED_MONOTONIC_HPP
#define ORCA_SHARED_MONOTONIC_HPP

#include "rclcpp/rclcpp.hpp"

namespace monotonic
{

  //=============================================================================
  // Common simulation problems:
  // -- msg.header.stamp might be 0
  // -- msg.header.stamp might repeat over consecutive messages
  //=============================================================================

  bool valid(const rclcpp::Time &t)
  {
    return t.nanoseconds() > 0;
  }

  template<typename NodeType, typename MsgType>
  class Valid
  {
    NodeType node_;
    std::function<void(NodeType, MsgType)> process_;         // Process good messages
    rclcpp::Time curr_;                         // Stamp of current message
    rclcpp::Time prev_;                         // Stamp of previous message

  public:

    Valid(NodeType node, std::function<void(NodeType, MsgType)> callback)
    {
      node_ = node;
      process_ = callback;
    }

    void call(MsgType msg)
    {
      curr_ = msg->header.stamp;

      if (valid(curr_)) {
        process_(node_, msg);
        prev_ = curr_;
      }
    }

    const rclcpp::Time &curr() const
    { return curr_; };

    const rclcpp::Time &prev() const
    { return prev_; };

    rclcpp::Duration d() const
    { return curr() - prev(); }

    double dt() const
    { return d().seconds(); }

    bool receiving() const
    { return valid(prev_); }
  };

  template<typename NodeType, typename MsgType>
  class Monotonic
  {
    NodeType node_;
    std::function<void(NodeType, MsgType, bool)> process_;   // Process good messages
    rclcpp::Time curr_{0, 0, RCL_ROS_TIME};
    rclcpp::Time prev_{0, 0, RCL_ROS_TIME};

  public:

    Monotonic(NodeType node, std::function<void(NodeType, MsgType, bool)> callback)
    {
      node_ = node;
      process_ = callback;
    }

    void call(MsgType msg)
    {
      curr_ = msg->header.stamp;

      if (valid(curr_)) {
        if (valid(prev_)) {
          // Must be monotonic
          if (curr_ > prev_) {
            process_(node_, msg, false);
            prev_ = curr_;
          }
        } else {
          process_(node_, msg, true);
          prev_ = curr_;
        }
      }
    }

    const rclcpp::Time &curr() const
    { return curr_; };

    const rclcpp::Time &prev() const
    { return prev_; };

    rclcpp::Duration d() const
    { return curr() - prev(); }

    double dt() const
    { return (curr() - prev()).seconds(); }

    bool receiving() const
    { return valid(prev_); }
  };

  template<typename NodeType>
  class Timer
  {
    NodeType node_;
    std::function<void(NodeType, bool)> process_;   // Process good messages
    rclcpp::Time curr_{0, 0, RCL_ROS_TIME};
    rclcpp::Time prev_{0, 0, RCL_ROS_TIME};

  public:

    Timer(NodeType node, std::function<void(NodeType, bool)> callback)
    {
      node_ = node;
      process_ = callback;
    }

    void call()
    {
      curr_ = node_->now();

      if (valid(curr_)) {
        if (valid(prev_)) {
          // Must be monotonic
          if (curr_ > prev_) {
            process_(node_, false);
            prev_ = curr_;
          }
        } else {
          process_(node_, true);
          prev_ = curr_;
        }
      }
    }

    const rclcpp::Time &curr() const
    { return curr_; };

    const rclcpp::Time &prev() const
    { return prev_; };

    rclcpp::Duration d() const
    { return curr() - prev(); }

    double dt() const
    { return (curr() - prev()).seconds(); }

    bool receiving() const
    { return valid(prev_); }
  };

} // namespace orca_shared

#endif //ORCA_SHARED_MONOTONIC_HPP
