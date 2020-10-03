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

#ifndef ORCA_SHARED__MW__ACCELERATION_HPP_
#define ORCA_SHARED__MW__ACCELERATION_HPP_

#include <ostream>

namespace mw
{

class Acceleration
{
  double x_{};
  double y_{};
  double z_{};
  double yaw_{};

public:
  Acceleration() = default;

  Acceleration(double x, double y, double z, double yaw)
  : x_{x},
    y_{y},
    z_{z},
    yaw_{yaw} {}

  double x() const
  {
    return x_;
  }

  double y() const
  {
    return y_;
  }

  double z() const
  {
    return z_;
  }

  double yaw() const
  {
    return yaw_;
  }

  double & x()
  {
    return x_;
  }

  double & y()
  {
    return y_;
  }

  double & z()
  {
    return z_;
  }

  double & yaw()
  {
    return yaw_;
  }

  void x(const double & v)
  {
    x_ = v;
  }

  void y(const double & v)
  {
    y_ = v;
  }

  void z(const double & v)
  {
    z_ = v;
  }

  void yaw(const double & v)
  {
    yaw_ = v;
  }

  Acceleration operator+(const Acceleration & that) const
  {
    return Acceleration{x_ + that.x_, y_ + that.y_, z_ + that.z_, yaw_ + that.yaw_};
  }

  Acceleration operator-(const Acceleration & that) const
  {
    return Acceleration{x_ - that.x_, y_ - that.y_, z_ - that.z_, yaw_ - that.yaw_};
  }

  Acceleration operator-() const
  {
    return Acceleration{-x_, -y_, -z_, -yaw_};
  }

  bool operator==(const Acceleration & that) const
  {
    return x_ == that.x_ &&
           y_ == that.y_ &&
           z_ == that.z_ &&
           yaw_ == that.yaw_;
  }

  bool operator!=(const Acceleration & that) const
  {
    return !(*this == that);
  }

  friend std::ostream & operator<<(std::ostream & os, const Acceleration & v);
};

}  // namespace mw

#endif  // ORCA_SHARED__MW__ACCELERATION_HPP_
