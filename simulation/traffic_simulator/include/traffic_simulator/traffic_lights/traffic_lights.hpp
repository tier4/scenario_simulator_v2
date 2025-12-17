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

#ifndef TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHTS_HPP_
#define TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHTS_HPP_

// This message will be deleted in the future
#if __has_include(<autoware_perception_msgs/msg/traffic_signal_array.hpp>)
#include <autoware_perception_msgs/msg/traffic_signal_array.hpp>
#endif

#if __has_include(<autoware_perception_msgs/msg/traffic_light_group_array.hpp>)
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#endif

#include <algorithm>
#include <set>
#include <traffic_simulator/traffic_lights/traffic_light_publisher.hpp>
#include <traffic_simulator/traffic_lights/traffic_lights_base.hpp>

namespace traffic_simulator
{
class ConventionalTrafficLights : public TrafficLightsBase
{
public:
  template <typename NodeTypePointer>
  explicit ConventionalTrafficLights(
    const NodeTypePointer & node_ptr, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils)
  : TrafficLightsBase(node_ptr, hdmap_utils),
    backward_compatible_publisher_ptr_(
      std::make_unique<TrafficLightPublisher<traffic_simulator_msgs::msg::TrafficLightArrayV1>>(
        node_ptr, "/simulation/traffic_lights"))
  {
  }

  ~ConventionalTrafficLights() override = default;

private:
  auto update() const -> void override
  {
    backward_compatible_publisher_ptr_->publish(
      clock_ptr_->now(), generateUpdateTrafficLightsRequest());
    if (isAnyTrafficLightChanged()) {
      marker_publisher_ptr_->deleteMarkers();
    }
    marker_publisher_ptr_->drawMarkers(traffic_lights_map_);
  }

  const std::unique_ptr<TrafficLightPublisherBase> backward_compatible_publisher_ptr_;
};

class V2ITrafficLights : public TrafficLightsBase
{
public:
  template <typename NodeTypePointer>
  explicit V2ITrafficLights(
    const NodeTypePointer & node_ptr, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils,
    const std::string & architecture_type)
  : TrafficLightsBase(node_ptr, hdmap_utils),
    publisher_ptr_(makePublisher(
      node_ptr, architecture_type,
      "/perception/traffic_light_recognition/external/traffic_signals")),
    legacy_topic_publisher_ptr_(makePublisher(node_ptr, architecture_type, "/v2x/traffic_signals"))
  {
  }

  ~V2ITrafficLights() override = default;

private:
  auto update() const -> void override
  {
    const auto now = clock_ptr_->now();
    const auto request = generateUpdateTrafficLightsRequest();
    publisher_ptr_->publish(now, request);
    legacy_topic_publisher_ptr_->publish(now, request);
    if (isAnyTrafficLightChanged()) {
      marker_publisher_ptr_->deleteMarkers();
    }
    marker_publisher_ptr_->drawMarkers(traffic_lights_map_);
  }

  template <typename NodeTypePointer>
  auto makePublisher(
    const NodeTypePointer & node_ptr, const std::string & architecture_type,
    const std::string & topic_name) -> std::unique_ptr<TrafficLightPublisherBase>
  {
    /*
       V2ITrafficLights in TrafficSimulator publishes using architecture-independent topics ("awf/universe..."):
       "/v2x/traffic_signals" and "/perception/traffic_light_recognition/external/traffic_signals"

       TrafficLightsDetector in SimpleSensorSimulator publishes using architecture-dependent topics:
       "/perception/traffic_light_recognition/internal/traffic_signals" for >= "awf/universe/20230906"
       "/perception/traffic_light_recognition/traffic_signals" for "awf/universe"
    */
    if (architecture_type == "awf/universe") {
      throw common::SemanticError(
        "This version of scenario_simulator_v2 does not support ", std::quoted(architecture_type),
        " as ", std::quoted("architecture_type"), ". Please use older version.");
#if __has_include(<autoware_perception_msgs/msg/traffic_signal_array.hpp>)
    } else if (architecture_type <= "awf/universe/20230906") {
      return std::make_unique<
        TrafficLightPublisher<autoware_perception_msgs::msg::TrafficSignalArray>>(
        node_ptr, topic_name);
#endif
#if __has_include(<autoware_perception_msgs/msg/traffic_light_group_array.hpp>)
    } else if (architecture_type >= "awf/universe/20240605") {
      return std::make_unique<
        TrafficLightPublisher<autoware_perception_msgs::msg::TrafficLightGroupArray>>(
        node_ptr, topic_name);
#endif
    } else {
      throw common::SemanticError(
        "Unexpected architecture_type ", std::quoted(architecture_type),
        " given for V2I traffic lights simulation.");
    }
  }

  const std::unique_ptr<TrafficLightPublisherBase> publisher_ptr_;
  const std::unique_ptr<TrafficLightPublisherBase> legacy_topic_publisher_ptr_;
};

class DetectedTrafficLights
{
public:
  explicit DetectedTrafficLights(const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils)
  : hdmap_utils_(hdmap_utils)
  {
  }

  auto setState(const lanelet::Id lanelet_id, const std::string & state) -> void
  {
    clearState(lanelet_id);
    addState(lanelet_id, state);
  }

  auto addState(const lanelet::Id lanelet_id, const std::string & state) -> void
  {
    auto [iter, inserted] =
      detected_traffic_lights_.try_emplace(lanelet_id, lanelet_id, *hdmap_utils_);
    iter->second.set(state);
  }

  auto clearState(const lanelet::Id lanelet_id) -> bool
  {
    return detected_traffic_lights_.erase(lanelet_id) > 0;
  }

  auto empty() const -> bool { return detected_traffic_lights_.empty(); }

  auto apply(simulation_api_schema::UpdateTrafficLightsRequest & request) const -> void
  {
    for (const auto & [lanelet_id, detected_light] : detected_traffic_lights_) {
      if (auto matched_state = std::find_if(
            request.mutable_states()->begin(), request.mutable_states()->end(),
            [lanelet_id](const auto & state) { return state.id() == lanelet_id; });
          matched_state != request.mutable_states()->end()) {
        *matched_state = static_cast<simulation_api_schema::TrafficSignal>(detected_light);
      } else {
        // add ground-truth-less detected traffic light
        *request.add_states() = static_cast<simulation_api_schema::TrafficSignal>(detected_light);
      }
    }
  }

private:
  std::map<lanelet::Id, TrafficLight> detected_traffic_lights_;

  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_;
};

template <typename GroundTruthType>
class TrafficLightsChannel
{
public:
  template <typename NodeTypePointer, typename... Args>
  explicit TrafficLightsChannel(
    const NodeTypePointer & node_ptr, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils,
    Args &&... args)
  : ground_truth_(
      std::make_shared<GroundTruthType>(node_ptr, hdmap_utils, std::forward<Args>(args)...)),
    detected_(std::make_shared<DetectedTrafficLights>(hdmap_utils))
  {
  }

  auto getGroundTruth() const -> std::shared_ptr<GroundTruthType> { return ground_truth_; }

  auto getDetected() const -> std::shared_ptr<DetectedTrafficLights> { return detected_; }

  auto hasDetectedChanges() const -> bool { return not detected_->empty(); }

  auto generateUpdateRequest() const -> simulation_api_schema::UpdateTrafficLightsRequest
  {
    auto request = ground_truth_->generateUpdateTrafficLightsRequest();
    detected_->apply(request);
    return request;
  }

private:
  std::shared_ptr<GroundTruthType> ground_truth_;

  std::shared_ptr<DetectedTrafficLights> detected_;
};

class TrafficLights
{
public:
  template <typename NodeTypePointer>
  explicit TrafficLights(
    const NodeTypePointer & node_ptr, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils,
    const std::string & architecture_type)
  : conventional_channel_(node_ptr, hdmap_utils),
    v2i_channel_(node_ptr, hdmap_utils, architecture_type),
    hdmap_utils_(hdmap_utils)
  {
  }

  auto isAnyTrafficLightChanged() -> bool;

  auto startTrafficLightsUpdate(
    const double conventional_traffic_light_update_rate,
    const double v2i_traffic_lights_update_rate) -> void;

  auto getConventionalTrafficLights() const -> std::shared_ptr<ConventionalTrafficLights>;

  auto getV2ITrafficLights() const -> std::shared_ptr<V2ITrafficLights>;

  auto getConventionalDetectedTrafficLights() const -> std::shared_ptr<DetectedTrafficLights>;

  auto getV2IDetectedTrafficLights() const -> std::shared_ptr<DetectedTrafficLights>;

  auto generateConventionalUpdateRequest() const
    -> simulation_api_schema::UpdateTrafficLightsRequest;

private:
  TrafficLightsChannel<ConventionalTrafficLights> conventional_channel_;

  TrafficLightsChannel<V2ITrafficLights> v2i_channel_;
};
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHTS_HPP_
