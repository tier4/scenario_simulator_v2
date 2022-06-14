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

#include <boost/algorithm/clamp.hpp>
#include <iostream>
#include <string>
#include <traffic_simulator/color_utils/color_utils.hpp>

namespace color_utils
{
void printRed(const std::string & text)
{
  std::string raw_text = "\x1b[31m" + text + "\x1b[0m\n";
  printf("%s", raw_text.c_str());
}

void printGreen(const std::string & text)
{
  std::string raw_text = "\x1b[32m" + text + "\x1b[0m\n";
  printf("%s", raw_text.c_str());
}

void printYellow(const std::string & text)
{
  std::string raw_text = "\x1b[33m" + text + "\x1b[0m\n";
  printf("%s", raw_text.c_str());
}

void printBlue(const std::string & text)
{
  std::string raw_text = "\x1b[34m" + text + "\x1b[0m\n";
  printf("%s", raw_text.c_str());
}

void printMagenta(const std::string & text)
{
  std::string raw_text = "\x1b[35m" + text + "\x1b[0m\n";
  printf("%s", raw_text.c_str());
}

void printCyan(const std::string & text)
{
  std::string raw_text = "\x1b[36m" + text + "\x1b[0m\n";
  printf("%s", raw_text.c_str());
}

void printWhite(const std::string & text)
{
  std::string raw_text = "\x1b[37m" + text + "\x1b[0m\n";
  printf("%s", raw_text.c_str());
}

/**
 * @brief generate std_msgs::msg::ColorRGBA message from rgb values
 * @param r red
 * @param g green
 * @param b blue
 * @param alpha alpha value of the color
 * @return std_msgs::msg::ColorRGBA
 */
const std_msgs::msg::ColorRGBA fromRgba(double r, double g, double b, double alpha)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = alpha;
  return color;
}
}  // namespace color_utils
