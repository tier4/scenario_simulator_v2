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

#ifndef GEOMETRY__TEST__VECTOR3__UTILS_HPP_
#define GEOMETRY__TEST__VECTOR3__UTILS_HPP_

#include <geometry_msgs/msg/vector3.hpp>

inline geometry_msgs::msg::Vector3 makeVector(double x, double y, double z = 0)
{
  geometry_msgs::msg::Vector3 v;
  v.x = x;
  v.y = y;
  v.z = z;
  return v;
}

#endif  // GEOMETRY__TEST__VECTOR3__UTILS_HPP_
