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

#ifndef ORCA_FILTER__PERF_HPP_
#define ORCA_FILTER__PERF_HPP_

#include <string>

#undef RUN_PERF
#ifdef RUN_PERF

#define NUM_SAMPLES 100

void print_mean(std::string msg, int samples[]);

#define START_PERF() \
  static int _samples_[NUM_SAMPLES]; \
  static int _index_ = 0; \
  auto _start_ = std::chrono::high_resolution_clock::now();

#define STOP_PERF(msg) \
  auto _stop_ = std::chrono::high_resolution_clock::now(); \
  _samples_[_index_] = \
    std::chrono::duration_cast<std::chrono::microseconds>(_stop_ - _start_).count(); \
  if (++_index_ >= NUM_SAMPLES) {print_mean(msg, _samples_); _index_ = 0;} \

#else
#define START_PERF()
#define STOP_PERF(msg)
#endif

#endif  // ORCA_FILTER__PERF_HPP_
