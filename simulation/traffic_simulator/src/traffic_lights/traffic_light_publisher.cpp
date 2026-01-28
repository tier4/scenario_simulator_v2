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

#include <simulation_interface/conversions.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_publisher.hpp>
#include <traffic_simulator_msgs/msg/traffic_light_array_v1.hpp>

// This message will be deleted in the future
#if __has_include(<autoware_perception_msgs/msg/traffic_signal_array.hpp>)
#include <autoware_perception_msgs/msg/traffic_signal_array.hpp>
#endif

#if __has_include(<autoware_perception_msgs/msg/traffic_light_group_array.hpp>)
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#endif

namespace traffic_simulator
{
template <typename T, typename = void>
struct HasMemberPredictions : std::false_type
{
};

template <typename T>
struct HasMemberPredictions<T, std::void_t<decltype(std::declval<T>().predictions)>>
: std::true_type
{
};

#if __has_include(<autoware_perception_msgs/msg/traffic_signal_array.hpp>)
template <>
auto TrafficLightPublisher<autoware_perception_msgs::msg::TrafficSignalArray>::generateMessage(
  const rclcpp::Time & current_ros_time,
  const simulation_api_schema::UpdateTrafficLightsRequest & request, const std::string &,
  const TrafficLightStatePredictions *)
  -> std::unique_ptr<autoware_perception_msgs::msg::TrafficSignalArray>
{
  auto message = std::make_unique<autoware_perception_msgs::msg::TrafficSignalArray>();

  message->stamp = current_ros_time;

  using TrafficLightType = autoware_perception_msgs::msg::TrafficSignal;
  using TrafficLightBulbType = TrafficLightType::_elements_type::value_type;

  for (const auto & traffic_light : request.states()) {
    for (const auto & relation_id : traffic_light.relation_ids()) {
      // skip if the traffic light has no bulbs
      if (not traffic_light.traffic_light_status().empty()) {
        TrafficLightType traffic_light_message;
        traffic_light_message.traffic_signal_id = relation_id;
        for (const auto & bulb_status : traffic_light.traffic_light_status()) {
          TrafficLightBulbType light_bulb_message;
          simulation_interface::toMsg<TrafficLightBulbType>(bulb_status, light_bulb_message);
          traffic_light_message.elements.push_back(light_bulb_message);
        }
        message->signals.push_back(traffic_light_message);
      }
    }
  }
  return message;
}
#endif  // __has_include(<autoware_perception_msgs/msg/traffic_signal_array.hpp>)

template <>
auto TrafficLightPublisher<traffic_simulator_msgs::msg::TrafficLightArrayV1>::generateMessage(
  const rclcpp::Time &, const simulation_api_schema::UpdateTrafficLightsRequest & request,
  const std::string &, const TrafficLightStatePredictions *)
  -> std::unique_ptr<traffic_simulator_msgs::msg::TrafficLightArrayV1>
{
  auto message = std::make_unique<traffic_simulator_msgs::msg::TrafficLightArrayV1>();

  using TrafficLightType = traffic_simulator_msgs::msg::TrafficLightV1;
  using TrafficLightBulbType = traffic_simulator_msgs::msg::TrafficLightBulbV1;

  for (const auto & traffic_light : request.states()) {
    TrafficLightType traffic_light_message;
    traffic_light_message.lanelet_way_id = traffic_light.id();
    for (const auto & bulb_status : traffic_light.traffic_light_status()) {
      TrafficLightBulbType light_bulb_message;
      simulation_interface::toMsg<TrafficLightBulbType>(bulb_status, light_bulb_message);
      traffic_light_message.traffic_light_bulbs.push_back(light_bulb_message);
    }
    message->traffic_lights.push_back(traffic_light_message);
  }
  return message;
}

#if __has_include(<autoware_perception_msgs/msg/traffic_light_group_array.hpp>)
template <>
auto TrafficLightPublisher<autoware_perception_msgs::msg::TrafficLightGroupArray>::generateMessage(
  const rclcpp::Time & current_ros_time,
  const simulation_api_schema::UpdateTrafficLightsRequest & request, const std::string &,
  const TrafficLightStatePredictions * predictions)
  -> std::unique_ptr<autoware_perception_msgs::msg::TrafficLightGroupArray>
{
  auto message = std::make_unique<autoware_perception_msgs::msg::TrafficLightGroupArray>();

  message->stamp = current_ros_time;

  using TrafficLightGroupType = autoware_perception_msgs::msg::TrafficLightGroup;
  using TrafficLightBulbType = autoware_perception_msgs::msg::TrafficLightElement;

  for (const auto & traffic_light : request.states()) {
    for (const auto & relation_id : traffic_light.relation_ids()) {
      // skip if the traffic light has no bulbs
      if (not traffic_light.traffic_light_status().empty()) {
        TrafficLightGroupType traffic_light_group_message;
        traffic_light_group_message.traffic_light_group_id = relation_id;
        for (auto bulb_status : traffic_light.traffic_light_status()) {
          TrafficLightBulbType light_bulb_message;
          simulation_interface::toMsg<TrafficLightBulbType>(bulb_status, light_bulb_message);
          traffic_light_group_message.elements.push_back(light_bulb_message);
        }

        if constexpr (HasMemberPredictions<TrafficLightGroupType>::value) {
          if (predictions) {
            auto prediction_phases = predictions->find(traffic_light.id());
            if (prediction_phases != predictions->end()) {
              using PredictedTrafficLightState =
                autoware_perception_msgs::msg::PredictedTrafficLightState;

              for (const auto & [stamp, bulbs] : prediction_phases->second) {
                auto prediction = PredictedTrafficLightState()
                                    .set__predicted_stamp(stamp)
                                    .set__information_source(
                                      PredictedTrafficLightState::INFORMATION_SOURCE_SIMULATION)
                                    .set__reliability(1.0);

                for (const auto & bulb_proto : bulbs) {
                  TrafficLightBulbType bulb_message;
                  simulation_interface::toMsg<TrafficLightBulbType>(bulb_proto, bulb_message);
                  prediction.simultaneous_elements.push_back(bulb_message);
                }
                traffic_light_group_message.predictions.push_back(prediction);
              }
            }
          }
        }

        message->traffic_light_groups.push_back(traffic_light_group_message);
      }
    }
  }
  return message;
}
#endif  // __has_include(<autoware_perception_msgs/msg/traffic_light_group_array.hpp>)
}  // namespace traffic_simulator
