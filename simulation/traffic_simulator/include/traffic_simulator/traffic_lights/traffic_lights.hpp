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

#include <traffic_simulator/traffic_lights/traffic_light_prediction.hpp>
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
  using PredictedState = TrafficLightPredictedState;

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

  auto setPrediction(
    const lanelet::Id lanelet_id, const std::string & state, double time_ahead_seconds) -> void
  {
    const auto now = clock_ptr_->now();
    PredictedState predicted_state;
    predicted_state.predicted_time = now + rclcpp::Duration(std::chrono::duration<double>(time_ahead_seconds));
    predicted_state.state = state;
    predicted_state.reliability = 1.0f;
    predicted_state.information_source = "SIMULATION";
    predictions_[lanelet_id].push_back(predicted_state);
  }

private:
  auto update() const -> void override
  {
    predictions_.clear();
    const auto now = clock_ptr_->now();
    const auto request = generateUpdateTrafficLightsRequest();
    publishWithPredictions(now, request);
    if (isAnyTrafficLightChanged()) {
      marker_publisher_ptr_->deleteMarkers();
    }
    marker_publisher_ptr_->drawMarkers(traffic_lights_map_);
  }

  auto publishWithPredictions(const rclcpp::Time & current_ros_time, const simulation_api_schema::UpdateTrafficLightsRequest & request) const -> void
  {
    // Use the new publish method that accepts predictions map pointer
    publisher_ptr_->publish(current_ros_time, request, &predictions_);
    legacy_topic_publisher_ptr_->publish(current_ros_time, request, &predictions_);
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

  mutable TrafficLightPredictions predictions_;
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
