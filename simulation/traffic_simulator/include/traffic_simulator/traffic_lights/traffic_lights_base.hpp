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

#ifndef TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHTS_BASE_HPP_
#define TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHTS_BASE_HPP_

#include <simulation_api_schema.pb.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/traffic_lights/configurable_rate_updater.hpp>
#include <traffic_simulator/traffic_lights/traffic_light.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_marker_publisher.hpp>
#include <unordered_map>
#include <vector>

namespace traffic_simulator
{
/*
   TrafficLightsBase class is designed in such a way that while trying to perform an operation
   on a TrafficLight (add, set, etc.) that is not added to traffic_light_map_,
   it adds the traffic light first and then performs the operation, so that the methods
   here cannot be tagged with the "const" specifier
*/
class TrafficLightsBase
{
public:
  template <typename NodeTypePointer>
  explicit TrafficLightsBase(
    const NodeTypePointer & node_ptr, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils)
  : hdmap_utils_(hdmap_utils),
    clock_ptr_(node_ptr->get_clock()),
    marker_publisher_ptr_(std::make_unique<TrafficLightMarkerPublisher>(node_ptr)),
    rate_updater_(node_ptr, [this]() { update(); })
  {
  }

  virtual ~TrafficLightsBase() = default;

  // update
  auto startUpdate(const double update_rate) -> void;

  auto resetUpdate(const double update_rate) -> void;

  // checks, setters, getters
  auto isAnyTrafficLightChanged() const -> bool;

  auto isRequiredStopTrafficLightState(const lanelet::Id traffic_light_id) -> bool;

  auto compareTrafficLightsState(const lanelet::Id lanelet_id, const std::string & state) -> bool;

  auto setTrafficLightsColor(
    const lanelet::Id lanelet_id, const traffic_simulator::TrafficLight::Color & color) -> void;

  auto setTrafficLightsState(const lanelet::Id lanelet_id, const std::string & state) -> void;

  auto setTrafficLightsConfidence(const lanelet::Id lanelet_id, const double confidence) -> void;

  auto getTrafficLightsComposedState(const lanelet::Id lanelet_id) -> std::string;

  auto getDistanceToActiveTrafficLightStopLine(
    const lanelet::Ids & route_lanelets, const math::geometry::CatmullRomSplineInterface & spline)
    -> std::optional<double>;

  auto generateUpdateTrafficLightsRequest() const
    -> simulation_api_schema::UpdateTrafficLightsRequest;

  auto getTrafficLight(const lanelet::Id traffic_light_id) -> TrafficLight &;

protected:
  virtual auto update() const -> void = 0;

  auto isTrafficLightAdded(const lanelet::Id traffic_light_id) const -> bool;

  auto addTrafficLight(const lanelet::Id traffic_light_id) -> void;

  auto getTrafficLights(const lanelet::Id lanelet_id)
    -> std::vector<std::reference_wrapper<TrafficLight>>;

  const std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_;
  const rclcpp::Clock::SharedPtr clock_ptr_;

  std::unordered_map<lanelet::Id, TrafficLight> traffic_lights_map_;
  const std::unique_ptr<TrafficLightMarkerPublisher> marker_publisher_ptr_;
  ConfigurableRateUpdater rate_updater_;
};
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHTS_BASE_HPP_
