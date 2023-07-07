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

#ifndef TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__CONFIGURABLE_RATE_UPDATER_HPP
#define TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__CONFIGURABLE_RATE_UPDATER_HPP

#include <rclcpp/rclcpp.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_manager_base.hpp>

namespace traffic_simulator
{
  class ConfigurableRateUpdater : public TrafficLightManagerBase  {
  public:
    template <typename NodePointer>
    ConfigurableRateUpdater(const NodePointer & node, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap,
    const std::string & map_frame = "map")
    : TrafficLightManagerBase(node, hdmap, map_frame) {

    }

//    virtual auto update() -> void = 0;
  };
}

#endif // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__CONFIGURABLE_RATE_UPDATER_HPP
