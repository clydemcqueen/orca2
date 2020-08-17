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

#ifndef ORCA_SHARED__MW__ACCELERATION_BODY_HPP_
#define ORCA_SHARED__MW__ACCELERATION_BODY_HPP_

#include <ostream>
#include <utility>

namespace mw
{

class AccelerationBody
{
  double forward_{};
  double strafe_{};
  double vertical_{};
  double yaw_{};

public:
  AccelerationBody() = default;

  AccelerationBody(double forward, double strafe, double vertical, double yaw)
  : forward_{forward},
    strafe_{strafe},
    vertical_{vertical},
    yaw_{yaw} {}

  double forward() const
  {
    return forward_;
  }

  double strafe() const
  {
    return strafe_;
  }

  double vertical() const
  {
    return vertical_;
  }

  double yaw() const
  {
    return yaw_;
  }

  double & forward()
  {
    return forward_;
  }

  double & strafe()
  {
    return strafe_;
  }

  double & vertical()
  {
    return vertical_;
  }

  double & yaw()
  {
    return yaw_;
  }

  void forward(const double & v)
  {
    forward_ = v;
  }

  void strafe(const double & v)
  {
    strafe_ = v;
  }

  void vertical(const double & v)
  {
    vertical_ = v;
  }

  void yaw(const double & v)
  {
    yaw_ = v;
  }

  AccelerationBody operator+(const AccelerationBody & that) const
  {
    return AccelerationBody{forward_ + that.forward_, strafe_ + that.strafe_,
      vertical_ + that.vertical_, yaw_ + that.yaw_};
  }

  AccelerationBody operator-(const AccelerationBody & that) const
  {
    return AccelerationBody{forward_ - that.forward_, strafe_ - that.strafe_,
      vertical_ - that.vertical_, yaw_ - that.yaw_};
  }

  AccelerationBody operator-() const
  {
    return AccelerationBody{-forward_, -strafe_, -vertical_, -yaw_};
  }

  bool operator==(const AccelerationBody & that) const
  {
    return forward_ == that.forward_ &&
           strafe_ == that.strafe_ &&
           vertical_ == that.vertical_ &&
           yaw_ == that.yaw_;
  }

  bool operator!=(const AccelerationBody & that) const
  {
    return !(*this == that);
  }

  friend std::ostream & operator<<(std::ostream & os, const AccelerationBody & v);
};

}  // namespace mw

#endif  // ORCA_SHARED__MW__ACCELERATION_BODY_HPP_
