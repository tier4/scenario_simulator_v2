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

#include <openscenario_interpreter/syntax/axle.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
Axle::operator openscenario_msgs::msg::Axle() const
{
  openscenario_msgs::msg::Axle axle;
  {
    axle.max_steering = max_steering;
    axle.wheel_diameter = wheel_diameter;
    axle.track_width = track_width;
    axle.position_x = position_x;
    axle.position_z = position_z;
  }

  return axle;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
