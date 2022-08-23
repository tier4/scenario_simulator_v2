// Copyright 2015 TIER IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
