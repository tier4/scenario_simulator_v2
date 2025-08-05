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
#include <type_traits>

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
  // TrafficSignalArray doesn't support predictions, so ignore the predictions parameter
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
  // V1 messages don't support predictions, so ignore the predictions parameter
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
  using autoware_perception_msgs::msg::PredictedTrafficLightState;
  using autoware_perception_msgs::msg::TrafficLightElement;
  using autoware_perception_msgs::msg::TrafficLightGroup;

  // NOTE: key is relation ID
  std::unordered_map<lanelet::Id, TrafficLightGroup> traffic_light_group_map;
  for (const auto & way_id_level_traffic_light : request.states()) {
    for (const auto & bulb_proto : way_id_level_traffic_light.traffic_light_status()) {
      TrafficLightElement bulb_message;
      simulation_interface::toMsg<TrafficLightElement>(bulb_proto, bulb_message);
      for (const auto & relation_id : way_id_level_traffic_light.relation_ids()) {
        traffic_light_group_map[relation_id].elements.push_back(bulb_message);
      }
    }
    if constexpr (HasMemberPredictions<TrafficLightGroup>::value) {
      if (predictions) {
        if (const auto prediction_phases = predictions->find(way_id_level_traffic_light.id());
            prediction_phases != predictions->end()) {
          for (const auto & [stamp, bulbs] : prediction_phases->second) {
            for (const auto & relation_id : way_id_level_traffic_light.relation_ids()) {
              auto prediction = [&]() -> std::reference_wrapper<PredictedTrafficLightState> {
                auto & predictions_message = traffic_light_group_map[relation_id].predictions;
                if (auto matched_prediction = std::find_if(
                      predictions_message.begin(), predictions_message.end(),
                      [&stamp](const auto & p) { return p.predicted_stamp == stamp; });
                    matched_prediction != predictions_message.end()) {
                  return *matched_prediction;
                } else {
                  return predictions_message.emplace_back(PredictedTrafficLightState()
                                                            .set__predicted_stamp(stamp)
                                                            .set__information_source("SIMULATION")
                                                            .set__reliability(1.0));
                }
              }();

              for (const auto & bulb_proto : bulbs) {
                TrafficLightElement bulb_message;
                simulation_interface::toMsg<TrafficLightElement>(bulb_proto, bulb_message);
                prediction.get().simultaneous_elements.push_back(bulb_message);
              }
            }
          }
        }
      }
    }
  }

  auto message = std::make_unique<autoware_perception_msgs::msg::TrafficLightGroupArray>();
  message->stamp = current_ros_time;

  for (const auto & [relation_id, traffic_light_group] : traffic_light_group_map) {
    message->traffic_light_groups.push_back(traffic_light_group);
    message->traffic_light_groups.back().traffic_light_group_id = relation_id;
  }
  return message;
}

#endif  // __has_include(<autoware_perception_msgs/msg/traffic_light_group_array.hpp>)
}  // namespace traffic_simulator
