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

#ifndef TRAFFIC_SIMULATOR__MATH__LINEAR_ALGEBRA_HPP_
#define TRAFFIC_SIMULATOR__MATH__LINEAR_ALGEBRA_HPP_

#include <math.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <limits>
#include <scenario_simulator_exception/exception.hpp>

namespace traffic_simulator
{
namespace math
{
geometry_msgs::msg::Vector3 vector3(double x, double y, double z);
double getSize(geometry_msgs::msg::Vector3 vec);
geometry_msgs::msg::Vector3 normalize(geometry_msgs::msg::Vector3 vec);
double innerProduct(const geometry_msgs::msg::Vector3 & v0, const geometry_msgs::msg::Vector3 & v1);
double getInternalAngle(
  const geometry_msgs::msg::Vector3 & v0, const geometry_msgs::msg::Vector3 & v1);
}  // namespace math
}  // namespace traffic_simulator

geometry_msgs::msg::Vector3 operator*(const geometry_msgs::msg::Vector3 & vec, double value);
geometry_msgs::msg::Vector3 operator*(double value, const geometry_msgs::msg::Vector3 & vec);
geometry_msgs::msg::Point operator+(
  const geometry_msgs::msg::Point & v0, const geometry_msgs::msg::Vector3 & v1);
geometry_msgs::msg::Vector3 operator+(
  const geometry_msgs::msg::Vector3 & v0, const geometry_msgs::msg::Vector3 & v1);
geometry_msgs::msg::Point operator-(
  const geometry_msgs::msg::Point & v0, const geometry_msgs::msg::Vector3 & v1);
geometry_msgs::msg::Vector3 operator-(
  const geometry_msgs::msg::Vector3 & v0, const geometry_msgs::msg::Vector3 & v1);

#endif  // TRAFFIC_SIMULATOR__MATH__LINEAR_ALGEBRA_HPP_
