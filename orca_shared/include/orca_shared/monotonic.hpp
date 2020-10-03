// Copyright (c) 2020, Clyde McQueen.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef ORCA_SHARED__MONOTONIC_HPP_
#define ORCA_SHARED__MONOTONIC_HPP_

#include "rclcpp/rclcpp.hpp"

namespace monotonic
{

//=============================================================================
// Common simulation problems:
// -- msg.header.stamp might be 0
// -- msg.header.stamp might repeat over consecutive messages
//=============================================================================

bool valid(const rclcpp::Time & t)
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
    } else {
      // std::cout << "valid: timestamp is 0" << std::endl;
    }
  }

  const rclcpp::Time & curr() const {return curr_;}

  const rclcpp::Time & prev() const {return prev_;}

  rclcpp::Duration d() const {return curr() - prev();}

  double dt() const {return d().seconds();}

  bool receiving() const {return valid(prev_);}
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
        } else {
          // std::cout << "monotonic: timestamp is out of order" << std::endl;
        }
      } else {
        process_(node_, msg, true);
        prev_ = curr_;
      }
    } else {
      // std::cout << "monotonic: timestamp is 0" << std::endl;
    }
  }

  const rclcpp::Time & curr() const {return curr_;}

  const rclcpp::Time & prev() const {return prev_;}

  rclcpp::Duration d() const {return curr() - prev();}

  double dt() const {return (curr() - prev()).seconds();}

  bool receiving() const {return valid(prev_);}
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

  const rclcpp::Time & curr() const {return curr_;}

  const rclcpp::Time & prev() const {return prev_;}

  rclcpp::Duration d() const {return curr() - prev();}

  double dt() const {return (curr() - prev()).seconds();}

  bool receiving() const {return valid(prev_);}
};

}  // namespace monotonic

#endif  // ORCA_SHARED__MONOTONIC_HPP_
