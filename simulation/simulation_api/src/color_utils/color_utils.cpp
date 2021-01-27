// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#include <simulation_api/color_utils/color_utils.hpp>

#include <iostream>
#include <string>

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

/**
 * @brief generate std_msgs::msg::ColorRGBA message from hsv values
 * @param h hue
 * @param s saturation
 * @param v value
 */
const std_msgs::msg::ColorRGBA fromHsv(double h, double s, double v, double alpha)
{
  std_msgs::msg::ColorRGBA color;
  color.a = alpha;
  float r = v;
  float g = v;
  float b = v;
  if (s > 0.0f) {
    h *= 6.0f;
    int i = static_cast<int>(h);
    float f = h - static_cast<float>(i);
    switch (i) {
      default:
      case 0:
        g *= 1 - s * (1 - f);
        b *= 1 - s;
        break;
      case 1:
        r *= 1 - s * f;
        b *= 1 - s;
        break;
      case 2:
        r *= 1 - s;
        b *= 1 - s * (1 - f);
        break;
      case 3:
        r *= 1 - s;
        g *= 1 - s * f;
        break;
      case 4:
        r *= 1 - s * (1 - f);
        g *= 1 - s;
        break;
      case 5:
        g *= 1 - s;
        b *= 1 - s * f;
        break;
    }
  }
  color.r = r;
  color.g = g;
  color.b = b;
  return color;
}

/**
 * @brief generate std_msgs::msg::ColorRGBA message from color name
 * @param preset_name the name of the color
 * @param alpha alpha value of the color
 * @return std_msgs::msg::ColorRGBA
 */
const std_msgs::msg::ColorRGBA makeColorMsg(std::string preset_name, double alpha)
{
  std_msgs::msg::ColorRGBA c_msg;
  c_msg.a = alpha;
  if (c_msg.a < 0.) {c_msg.a = 0.;}
  if (c_msg.a > 1.) {c_msg.a = 1.;}

  auto found_itr = COLOR_NAME_DICT.find(preset_name);
  if (found_itr != COLOR_NAME_DICT.end()) {
    c_msg.r = found_itr->second[0];
    c_msg.g = found_itr->second[1];
    c_msg.b = found_itr->second[2];
  } else {
    c_msg.r = 0;
    c_msg.g = 0;
    c_msg.b = 0;
  }
  return c_msg;
}
}  // namespace color_utils
