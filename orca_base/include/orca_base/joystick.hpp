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

#ifndef ORCA_BASE__JOYSTICK_HPP_
#define ORCA_BASE__JOYSTICK_HPP_

// XBox One joystick axes and buttons
const int JOY_AXIS_LEFT_LR = 0;         // Left stick left/right; 1.0 is left and -1.0 is right
const int JOY_AXIS_LEFT_FB = 1;         // Left stick forward/back; 1.0 is forward and -1.0 is back
const int JOY_AXIS_LEFT_TRIGGER = 2;    // Left trigger
const int JOY_AXIS_RIGHT_LR = 3;        /* Right stick left/right; 1.0 is left and -1.0 is right*/
const int JOY_AXIS_RIGHT_FB = 4;        // Right stick forward/back; 1.0 is forward and -1.0 is back
const int JOY_AXIS_RIGHT_TRIGGER = 5;   // Right trigger
const int JOY_AXIS_TRIM_LR = 6;         // Trim left/right; 1.0 for left and -1.0 for right
const int JOY_AXIS_TRIM_FB = 7;         // Trim forward/back; 1.0 for forward and -1.0 for back
const int JOY_BUTTON_A = 0;             // A button
const int JOY_BUTTON_B = 1;             // B button
const int JOY_BUTTON_X = 2;             // X button
const int JOY_BUTTON_Y = 3;             // Y button
const int JOY_BUTTON_LEFT_BUMPER = 4;   // Left bumper
const int JOY_BUTTON_RIGHT_BUMPER = 5;  // Right bumper
const int JOY_BUTTON_VIEW = 6;          // View button
const int JOY_BUTTON_MENU = 7;          // Menu button
const int JOY_BUTTON_LOGO = 8;          // XBox logo button
const int JOY_BUTTON_LEFT_STICK = 9;    // Left stick button
const int JOY_BUTTON_RIGHT_STICK = 10;  // Right stick button

#endif  // ORCA_BASE__JOYSTICK_HPP_
