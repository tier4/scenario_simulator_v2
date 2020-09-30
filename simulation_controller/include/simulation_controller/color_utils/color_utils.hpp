// Copyright (c) 2019 shuhei yoshida
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

/**
 * @file color_util.h
 * @brief Preset Color names
 */

#ifndef SIMULATION_CONTROLLER__COLOR_UTILS__COLOR_UTILS_HPP_
#define SIMULATION_CONTROLLER__COLOR_UTILS__COLOR_UTILS_HPP_

#include <std_msgs/msg/color_rgba.hpp>

#include <cassert>
#include <map>
#include <string>

namespace color_utils
{
std_msgs::msg::ColorRGBA makeColorMsg(std::string preset_name, double alpha = 1.0);
/**
   * @brief 0 <= h <= 1.0,0 <= s <= 1.0,0 <= v <= 1.0
   */
std_msgs::msg::ColorRGBA fromHsv(double h, double s, double v, double alpha = 1.0);

/// @todo Read data from text data?
const std::map<std::string, std::array<float, 3>> COLOR_NAME_DICT{
  // {"COLOR_NAME", {R, G, B}} // template
  {"aliceblue", {0.941176, 0.972549, 1}},
  {"antiquewhite", {0.980392, 0.921569, 0.843137}},
  {"aqua", {0, 1, 1}},
  {"aquamarine", {0.498039, 1, 0.831373}},
  {"azure", {0.941176, 1, 1}},
  {"beige", {0.960784, 0.960784, 0.862745}},
  {"bisque", {1, 0.894118, 0.768627}},
  {"black", {0, 0, 0}},
  {"blanchedalmond", {1, 0.921569, 0.803922}},
  {"blue", {0, 0, 1}},
  {"blueviolet", {0.541176, 0.168627, 0.886275}},
  {"brown", {0.647059, 0.164706, 0.164706}},
  {"burlywood", {0.870588, 0.721569, 0.529412}},
  {"cadetblue", {0.372549, 0.619608, 0.627451}},
  {"chartreuse", {0.498039, 1, 0}},
  {"chocolate", {0.823529, 0.411765, 0.117647}},
  {"coral", {1, 0.498039, 0.313725}},
  {"cornflowerblue", {0.392157, 0.584314, 0.929412}},
  {"cornsilk", {1, 0.972549, 0.862745}},
  {"crimson", {0.862745, 0.0784314, 0.235294}},
  {"cyan", {0, 1, 1}},
  {"darkblue", {0, 0, 0.545098}},
  {"darkcyan", {0, 0.545098, 0.545098}},
  {"darkgoldenrod", {0.721569, 0.52549, 0.0431373}},
  {"darkgray", {0.662745, 0.662745, 0.662745}},
  {"darkgreen", {0, 0.392157, 0}},
  {"darkkhaki", {0.741176, 0.717647, 0.419608}},
  {"darkmagenta", {0.545098, 0, 0.545098}},
  {"darkolivegreen", {0.333333, 0.419608, 0.184314}},
  {"darkorange", {1, 0.54902, 0}},
  {"darkorchid", {0.6, 0.196078, 0.8}},
  {"darkred", {0.545098, 0, 0}},
  {"darksalmon", {0.913725, 0.588235, 0.478431}},
  {"darkseagreen", {0.560784, 0.737255, 0.560784}},
  {"darkslateblue", {0.282353, 0.239216, 0.545098}},
  {"darkslategray", {0.184314, 0.309804, 0.309804}},
  {"darkturquoise", {0, 0.807843, 0.819608}},
  {"darkviolet", {0.580392, 0, 0.827451}},
  {"deeppink", {1, 0.0784314, 0.576471}},
  {"deepskyblue", {0, 0.74902, 1}},
  {"dimgray", {0.411765, 0.411765, 0.411765}},
  {"dodgerblue", {0.117647, 0.564706, 1}},
  {"firebrick", {0.698039, 0.133333, 0.133333}},
  {"floralwhite", {1, 0.980392, 0.941176}},
  {"forestgreen", {0.133333, 0.545098, 0.133333}},
  {"fuchsia", {1, 0, 1}},
  {"gainsboro", {0.862745, 0.862745, 0.862745}},
  {"ghostwhite", {0.972549, 0.972549, 1}},
  {"gold", {1, 0.843137, 0}},
  {"goldenrod", {0.854902, 0.647059, 0.12549}},
  {"gray", {0.501961, 0.501961, 0.501961}},
  {"green", {0, 0.501961, 0}},
  {"greenyellow", {0.678431, 1, 0.184314}},
  {"honeydew", {0.941176, 1, 0.941176}},
  {"hotpink", {1, 0.411765, 0.705882}},
  {"indianred", {0.803922, 0.360784, 0.360784}},
  {"indigo", {0.294118, 0, 0.509804}},
  {"ivory", {1, 1, 0.941176}},
  {"khaki", {0.941176, 0.901961, 0.54902}},
  {"lavender", {0.901961, 0.901961, 0.980392}},
  {"lavenderblush", {1, 0.941176, 0.960784}},
  {"lawngreen", {0.486275, 0.988235, 0}},
  {"lemonchiffon", {1, 0.980392, 0.803922}},
  {"lightblue", {0.678431, 0.847059, 0.901961}},
  {"lightcoral", {0.941176, 0.501961, 0.501961}},
  {"lightcyan", {0.878431, 1, 1}},
  {"lightgoldenrodyellow", {0.980392, 0.980392, 0.823529}},
  {"lightgray", {0.827451, 0.827451, 0.827451}},
  {"lightgreen", {0.564706, 0.933333, 0.564706}},
  {"lightpink", {1, 0.713725, 0.756863}},
  {"lightsalmon", {1, 0.627451, 0.478431}},
  {"lightseagreen", {0.12549, 0.698039, 0.666667}},
  {"lightskyblue", {0.529412, 0.807843, 0.980392}},
  {"lightslategray", {0.466667, 0.533333, 0.6}},
  {"lightsteelblue", {0.690196, 0.768627, 0.870588}},
  {"lightyellow", {1, 1, 0.878431}},
  {"lime", {0, 1, 0}},
  {"limegreen", {0.196078, 0.803922, 0.196078}},
  {"linen", {0.980392, 0.941176, 0.901961}},
  {"magenta", {1, 0, 1}},
  {"maroon", {0.501961, 0, 0}},
  {"mediumaquamarine", {0.4, 0.803922, 0.666667}},
  {"mediumblue", {0, 0, 0.803922}},
  {"mediumorchid", {0.729412, 0.333333, 0.827451}},
  {"mediumpurple", {0.576471, 0.439216, 0.858824}},
  {"mediumseagreen", {0.235294, 0.701961, 0.443137}},
  {"mediumslateblue", {0.482353, 0.407843, 0.933333}},
  {"mediumspringgreen", {0, 0.980392, 0.603922}},
  {"mediumturquoise", {0.282353, 0.819608, 0.8}},
  {"mediumvioletred", {0.780392, 0.0823529, 0.521569}},
  {"midnightblue", {0.0980392, 0.0980392, 0.439216}},
  {"mintcream", {0.960784, 1, 0.980392}},
  {"mistyrose", {1, 0.894118, 0.882353}},
  {"moccasin", {1, 0.894118, 0.709804}},
  {"navajowhite", {1, 0.870588, 0.678431}},
  {"navy", {0, 0, 0.501961}},
  {"oldlace", {0.992157, 0.960784, 0.901961}},
  {"olive", {0.501961, 0.501961, 0}},
  {"olivedrab", {0.419608, 0.556863, 0.137255}},
  {"orange", {1, 0.647059, 0}},
  {"orangered", {1, 0.270588, 0}},
  {"orchid", {0.854902, 0.439216, 0.839216}},
  {"palegoldenrod", {0.933333, 0.909804, 0.666667}},
  {"palegreen", {0.596078, 0.984314, 0.596078}},
  {"paleturquoise", {0.686275, 0.933333, 0.933333}},
  {"palevioletred", {0.858824, 0.439216, 0.576471}},
  {"papayawhip", {1, 0.937255, 0.835294}},
  {"peachpuff", {1, 0.854902, 0.72549}},
  {"peru", {0.803922, 0.521569, 0.247059}},
  {"pink", {1, 0.752941, 0.796078}},
  {"plum", {0.866667, 0.627451, 0.866667}},
  {"powderblue", {0.690196, 0.878431, 0.901961}},
  {"purple", {0.501961, 0, 0.501961}},
  {"red", {1, 0, 0}},
  {"rosybrown", {0.737255, 0.560784, 0.560784}},
  {"royalblue", {0.254902, 0.411765, 0.882353}},
  {"saddlebrown", {0.545098, 0.270588, 0.0745098}},
  {"salmon", {0.980392, 0.501961, 0.447059}},
  {"sandybrown", {0.956863, 0.643137, 0.376471}},
  {"seagreen", {0.180392, 0.545098, 0.341176}},
  {"seashell", {1, 0.960784, 0.933333}},
  {"sienna", {0.627451, 0.321569, 0.176471}},
  {"silver", {0.752941, 0.752941, 0.752941}},
  {"skyblue", {0.529412, 0.807843, 0.921569}},
  {"slateblue", {0.415686, 0.352941, 0.803922}},
  {"slategray", {0.439216, 0.501961, 0.564706}},
  {"snow", {1, 0.980392, 0.980392}},
  {"springgreen", {0, 1, 0.498039}},
  {"steelblue", {0.27451, 0.509804, 0.705882}},
  {"tan", {0.823529, 0.705882, 0.54902}},
  {"teal", {0, 0.501961, 0.501961}},
  {"thistle", {0.847059, 0.74902, 0.847059}},
  {"tomato", {1, 0.388235, 0.278431}},
  {"turquoise", {0.25098, 0.878431, 0.815686}},
  {"violet", {0.933333, 0.509804, 0.933333}},
  {"wheat", {0.960784, 0.870588, 0.701961}},
  {"white", {1, 1, 1}},
  {"whitesmoke", {0.960784, 0.960784, 0.960784}},
  {"yellow", {1, 1, 0}},
  {"yellowgreen", {0.603922, 0.803922, 0.196078}},
  {"ERROR", {0, 0, 0}}};

}  // namespace color_utils

#endif  // SIMULATION_CONTROLLER__COLOR_UTILS__COLOR_UTILS_HPP_
