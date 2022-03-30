// MIT License
//
// Copyright (c) 2019 Shuhei-YOSHIDA
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/**
 * @file color_util.h
 * @brief Preset Color names
 */

#ifndef TRAFFIC_SIMULATOR__COLOR_UTILS__COLOR_UTILS_HPP_
#define TRAFFIC_SIMULATOR__COLOR_UTILS__COLOR_UTILS_HPP_

#include <cassert>
#include <cstdio>
#include <map>
#include <std_msgs/msg/color_rgba.hpp>
#include <string>

namespace color_utils
{
void printRed(const std::string & text);
void printGreen(const std::string & text);
void printYellow(const std::string & text);
void printBlue(const std::string & text);
void printMagenta(const std::string & text);
void printCyan(const std::string & text);
void printWhite(const std::string & text);

const std_msgs::msg::ColorRGBA fromRgba(double r, double g, double b, double alpha = 1.0);
}  // namespace color_utils

#endif  // TRAFFIC_SIMULATOR__COLOR_UTILS__COLOR_UTILS_HPP_
