// Copyright 2024 TIER IV, Inc. All rights reserved.
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

#ifndef TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_SUPERVISOR_HPP_
#define TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_SUPERVISOR_HPP_

#include <iomanip>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <simulation_interface/conversions.hpp>
#include <stdexcept>  // std::out_of_range
#include <string>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/traffic_lights/configurable_rate_updater.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_manager.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_marker_publisher.hpp>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace traffic_simulator
{
class TrafficLightSupervisor
{
public:
  template <typename NodePointerT>
  explicit TrafficLightSupervisor(
    const NodePointerT & node, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils,
    const std::string & architecture_type)
  : clock_ptr_(node->get_clock()),
    conventional_traffic_light_manager_ptr_(std::make_shared<TrafficLightManager>(hdmap_utils)),
    conventional_traffic_light_marker_publisher_ptr_(
      std::make_shared<TrafficLightMarkerPublisher>(conventional_traffic_light_manager_ptr_, node)),
    v2i_traffic_light_manager_ptr_(std::make_shared<TrafficLightManager>(hdmap_utils)),
    v2i_traffic_light_marker_publisher_ptr_(
      std::make_shared<TrafficLightMarkerPublisher>(v2i_traffic_light_manager_ptr_, node)),
    v2i_traffic_light_legacy_topic_publisher_ptr_(
      makeV2ITrafficLightPublisher(architecture_type, "/v2x/traffic_signals", node, hdmap_utils)),
    v2i_traffic_light_publisher_ptr_(makeV2ITrafficLightPublisher(
      architecture_type, "/perception/traffic_light_recognition/external/traffic_signals", node,
      hdmap_utils)),
    v2i_traffic_light_updater_(
      node,
      [this]() {
        v2i_traffic_light_marker_publisher_ptr_->publish();
        v2i_traffic_light_publisher_ptr_->publish(
          clock_ptr_->now(), v2i_traffic_light_manager_ptr_->generateUpdateTrafficLightsRequest());
        v2i_traffic_light_legacy_topic_publisher_ptr_->publish(
          clock_ptr_->now(), v2i_traffic_light_manager_ptr_->generateUpdateTrafficLightsRequest());
      }),
    conventional_traffic_light_updater_(
      node, [this]() { conventional_traffic_light_marker_publisher_ptr_->publish(); })
  {
  }

  auto getConventionalTrafficLightManager() const -> std::shared_ptr<TrafficLightManager>
  {
    return conventional_traffic_light_manager_ptr_;
  }

  auto getV2ITrafficLightManager() const -> std::shared_ptr<TrafficLightManager>
  {
    return v2i_traffic_light_manager_ptr_;
  }

  auto resetConventionalTrafficLightPublishRate(double rate) -> void
  {
    conventional_traffic_light_updater_.resetUpdateRate(rate);
  }

  auto resetV2ITrafficLightPublishRate(double rate) -> void
  {
    v2i_traffic_light_updater_.resetUpdateRate(rate);
  }

  auto setConventionalTrafficLightConfidence(lanelet::Id id, double confidence) -> void
  {
    for (auto & traffic_light : conventional_traffic_light_manager_ptr_->getTrafficLights(id)) {
      traffic_light.get().confidence = confidence;
    }
  }

  auto trafficLightsChanged() -> bool
  {
    return conventional_traffic_light_manager_ptr_->hasAnyLightChanged() or
           v2i_traffic_light_manager_ptr_->hasAnyLightChanged();
  }

  auto createConventionalTimer(double update_rate) -> void
  {
    conventional_traffic_light_updater_.createTimer(update_rate);
  }

  auto createV2ITimer(double update_rate) -> void
  {
    v2i_traffic_light_updater_.createTimer(update_rate);
  }

protected:
  template <typename... Ts>
  auto makeV2ITrafficLightPublisher(const std::string & architecture_type, Ts &&... xs)
    -> std::shared_ptr<TrafficLightPublisherBase>
  {
    if (architecture_type.find("awf/universe") != std::string::npos) {
      return std::make_shared<
        TrafficLightPublisher<autoware_perception_msgs::msg::TrafficSignalArray>>(
        std::forward<decltype(xs)>(xs)...);
    } else {
      throw common::SemanticError(
        "Unexpected architecture_type ", std::quoted(architecture_type),
        " given for V2I traffic lights simulation.");
    }
  }

protected:
  const rclcpp::Clock::SharedPtr clock_ptr_;

  const std::shared_ptr<TrafficLightManager> conventional_traffic_light_manager_ptr_;
  const std::shared_ptr<TrafficLightMarkerPublisher>
    conventional_traffic_light_marker_publisher_ptr_;

  const std::shared_ptr<TrafficLightManager> v2i_traffic_light_manager_ptr_;
  const std::shared_ptr<TrafficLightMarkerPublisher> v2i_traffic_light_marker_publisher_ptr_;
  const std::shared_ptr<TrafficLightPublisherBase> v2i_traffic_light_legacy_topic_publisher_ptr_;
  const std::shared_ptr<TrafficLightPublisherBase> v2i_traffic_light_publisher_ptr_;

  ConfigurableRateUpdater v2i_traffic_light_updater_, conventional_traffic_light_updater_;
};
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_SUPERVISOR_HPP_
