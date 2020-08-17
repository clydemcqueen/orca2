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

#include "orca_driver/maestro.hpp"

#include <termios.h>
#include <unistd.h>

#include <fcntl.h>
#include <string>

namespace maestro
{

Maestro::Maestro()
: fd_{-1}
{
}

Maestro::~Maestro()
{
  if (ready()) {
    disconnect();
  }
}

// Open the virtual serial port, return true if successful
bool Maestro::connect(const std::string & port)
{
  fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ == -1) {
    // Likely causes of failure: (a) we're not root, (b) wrong port
    return false;
  } else {
    struct termios port_settings {};
    tcgetattr(fd_, &port_settings);
    port_settings.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    port_settings.c_oflag &= ~(ONLCR | OCRNL);
    tcsetattr(fd_, TCSANOW, &port_settings);

    return true;
  }
}

// Close the virtual serial port
void Maestro::disconnect()
{
  close(fd_);
  fd_ = -1;
}

// Return true if the port is open
bool Maestro::ready() const
{
  return fd_ != -1;
}

// Set the servo / ESC PWM signal, value is in microseconds, return true if successful
bool Maestro::setPWM(uint8_t channel, uint16_t value)
{
  if (ready()) {
    value *= 4;  // Maestro units are 0.25us, e.g., 1500us becomes 6000qus
    uint8_t cmd[4] = {0x84, channel, static_cast<uint8_t>(value & 0x7F),
      static_cast<uint8_t>((value >> 7) & 0x7F)};
    return writeBytes(cmd, sizeof(cmd));
  } else {
    return false;
  }
}

// Get the value at a particular channel
bool Maestro::getValue(uint8_t channel, uint16_t & value)
{
  if (ready()) {
    uint8_t cmd[2] = {0x90, channel};
    if (!writeBytes(cmd, sizeof(cmd))) {
      return false;
    }
    uint8_t response[2] = {0x00, 0x00};
    if (!readBytes(response, sizeof(response))) {
      return false;
    }
    value = response[0] + static_cast<uint16_t>(256 * response[1]);
    return true;
  } else {
    return false;
  }
}

// Get the servo / ESC PWM signal, value is in microseconds, return true if successful
bool Maestro::getPWM(uint8_t channel, uint16_t & value)
{
  if (!getValue(channel, value)) {
    return false;
  }

  // Maestro pwm measurements are in 0.25us
  value /= 4;
  return true;
}

// Get the value of an analog pin, 0-5.0V
bool Maestro::getAnalog(uint8_t channel, double & value)
{
  uint16_t temp;
  if (!getValue(channel, temp)) {
    return false;
  }

  // Maestro analog measurements are 0-1023, mapped to 0-5.0V
  value = temp * 5.0 / 1023;
  return true;
}

// Get the value of a digital pin, true = high
bool Maestro::getDigital(uint8_t channel, bool & value)
{
  uint16_t temp;
  if (!getValue(channel, temp)) {
    return false;
  }

  // Maestro digital measurements are 1023=high, everything else low
  value = (temp == 1023);
  return true;
}

// Write bytes to the serial port, return true if successful
bool Maestro::writeBytes(const uint8_t * bytes, ssize_t size) const
{
  return ready() && write(fd_, bytes, size) == size;
}

// Read bytes from the serial port, return true if successful
bool Maestro::readBytes(uint8_t * bytes, ssize_t size) const
{
  return ready() && read(fd_, bytes, size) == size;
}

}  // namespace maestro
