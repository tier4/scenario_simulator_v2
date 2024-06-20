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

#ifndef TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHTS_HPP_
#define TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHTS_HPP_

#include <autoware_auto_perception_msgs/msg/traffic_signal_array.hpp>
#include <autoware_perception_msgs/msg/traffic_signal_array.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_publisher.hpp>
#include <traffic_simulator/traffic_lights/traffic_lights_base.hpp>

namespace traffic_simulator
{
class ConventionalTrafficLights : public TrafficLightsBase
{
public:
  template <typename NodeTypePointer>
  ConventionalTrafficLights(
    const NodeTypePointer & node_ptr, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils)
  : TrafficLightsBase(node_ptr, hdmap_utils)
  {
  }
};

class V2ITrafficLights : public TrafficLightsBase
{
public:
  template <typename NodeTypePointer>
  V2ITrafficLights(
    const NodeTypePointer & node_ptr, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils,
    const std::string & architecture_type)
  : TrafficLightsBase(node_ptr, hdmap_utils),
    publisher_ptr_(makePublisher(
      architecture_type, "/perception/traffic_light_recognition/external/traffic_signals", node_ptr,
      hdmap_utils)),
    legacy_topic_publisher_ptr_(
      makePublisher(architecture_type, "/v2x/traffic_signals", node_ptr, hdmap_utils))
  {
  }

private:
  auto update() const -> void override
  {
    TrafficLightsBase::update();
    publisher_ptr_->publish(clock_ptr_->now(), generateUpdateTrafficLightsRequest());
    legacy_topic_publisher_ptr_->publish(clock_ptr_->now(), generateUpdateTrafficLightsRequest());
  }

  template <typename... Ts>
  auto makePublisher(const std::string & architecture_type, Ts &&... xs)
    -> std::unique_ptr<TrafficLightPublisherBase>
  {
    /// here autoware_perception_msgs is used for all awf/universe/**.
    /// @note perhaps autoware_auto_perception_msgs should be used for >= "awf/universe/20230906"?
    if (architecture_type.find("awf/universe") != std::string::npos) {
      return std::make_unique<
        TrafficLightPublisher<autoware_perception_msgs::msg::TrafficSignalArray>>(
        std::forward<decltype(xs)>(xs)...);
    } else {
      throw common::SemanticError(
        "Unexpected architecture_type ", std::quoted(architecture_type),
        " given for V2I traffic lights simulation.");
    }
  }

  const std::unique_ptr<TrafficLightPublisherBase> publisher_ptr_;
  const std::unique_ptr<TrafficLightPublisherBase> legacy_topic_publisher_ptr_;
};

class TrafficLights
{
public:
  template <typename NodeTypePointer>
  explicit TrafficLights(
    const NodeTypePointer & node_ptr, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils,
    const std::string & architecture_type)
  : conventional_traffic_lights_(
      std::make_shared<ConventionalTrafficLights>(node_ptr, hdmap_utils)),
    v2i_traffic_lights_(
      std::make_shared<V2ITrafficLights>(node_ptr, hdmap_utils, architecture_type))
  {
  }

  auto isAnyTrafficLightChanged() -> bool;

  auto startTrafficLightsUpdate(
    const double conventional_traffic_light_update_rate,
    const double v2i_traffic_lights_update_rate) -> void;

  auto getConventionalTrafficLights() const -> std::shared_ptr<ConventionalTrafficLights>;

  auto getV2ITrafficLights() const -> std::shared_ptr<V2ITrafficLights>;

private:
  const std::shared_ptr<ConventionalTrafficLights> conventional_traffic_lights_;
  const std::shared_ptr<V2ITrafficLights> v2i_traffic_lights_;
};
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHTS_HPP_
