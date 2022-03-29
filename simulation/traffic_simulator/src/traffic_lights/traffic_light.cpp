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

#include <limits>
#include <scenario_simulator_exception/exception.hpp>
#include <string>
#include <traffic_simulator/color_utils/color_utils.hpp>
#include <traffic_simulator/traffic_lights/traffic_light.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

namespace traffic_simulator
{
const geometry_msgs::msg::Point & TrafficLight::getPosition(const TrafficLightColor & color) const
{
  if (color_positions_.count(color) == 0) {
    THROW_SEMANTIC_ERROR("target color does not exist");
  }
  return color_positions_.at(color);
}

const geometry_msgs::msg::Point & TrafficLight::getPosition(const TrafficLightArrow & arrow) const
{
  if (arrow_positions_.count(arrow) == 0) {
    THROW_SEMANTIC_ERROR("target arrow does not exist");
  }
  return arrow_positions_.at(arrow);
}
}  // namespace traffic_simulator
