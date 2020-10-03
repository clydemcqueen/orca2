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

#ifndef ORCA_DRIVER__MAESTRO_HPP_
#define ORCA_DRIVER__MAESTRO_HPP_

#include <string>

namespace maestro
{

// A very simple class to communicate w/ the Pololu Maestro board via USB.

class Maestro
{
private:
  int fd_;

  bool writeBytes(const uint8_t * bytes, ssize_t size) const;

  bool readBytes(uint8_t * bytes, ssize_t size) const;

  bool getValue(uint8_t channel, uint16_t & value);

public:
  Maestro();

  ~Maestro();

  bool connect(const std::string & port);

  void disconnect();

  bool ready() const;

  bool setPWM(uint8_t channel, uint16_t value);

  bool getPWM(uint8_t channel, uint16_t & value);

  bool getAnalog(uint8_t channel, double & value);

  bool getDigital(uint8_t channel, bool & value);
};

}  // namespace maestro

#endif  // ORCA_DRIVER__MAESTRO_HPP_
