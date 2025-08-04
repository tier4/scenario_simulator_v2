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

#include <traffic_simulator/traffic_lights/traffic_lights.hpp>

namespace traffic_simulator
{
  auto V2ITrafficLights::setTrafficLightsStatePrediction(const lanelet::Id lanelet_id, const std::string &state,
    double time_ahead_seconds) -> void {
    std::cout << "setTrafficLightsStatePrediction: " << lanelet_id << ", " << state << ", " << time_ahead_seconds << std::endl;
    // TrafficLightのvectorを作成
    std::vector<simulation_api_schema::TrafficLight> traffic_light_states;
    
    // 既存のTrafficLightを取得してstateを解析
    auto traffic_lights = getTrafficLights(lanelet_id);
    if (!traffic_lights.empty()) {
      auto & traffic_light = traffic_lights.front().get();
      TrafficLight temp_traffic_light = traffic_light;
      temp_traffic_light.set(state);
      
      // 各bulbをsimulation_api_schema::TrafficLightに変換
      for (const auto & bulb : temp_traffic_light.bulbs) {
        auto traffic_light_proto = static_cast<simulation_api_schema::TrafficLight>(bulb);
        traffic_light_proto.set_confidence(temp_traffic_light.confidence);
        traffic_light_states.push_back(traffic_light_proto);
      }
    }
    
    // 予測時刻を計算
    const auto predicted_time = clock_ptr_->now() + rclcpp::Duration(std::chrono::duration<double>(time_ahead_seconds));
    
    // 同じ時刻のエントリがあるか確認
    auto & predictions_for_id = predictions_[lanelet_id];
    auto it = std::find_if(predictions_for_id.begin(), predictions_for_id.end(),
                           [&predicted_time](const auto & pair) {
                             // 時刻の差が小さい場合は同じ時刻とみなす（1ミリ秒以内）
                             return std::abs((pair.first - predicted_time).seconds()) < 0.001;
                           });
    
    if (it != predictions_for_id.end()) {
      // 同じ時刻のエントリが存在する場合は追加
      for (const auto & traffic_light_state : traffic_light_states) {
        it->second.push_back(traffic_light_state);
      }
    } else {
      std::pair<rclcpp::Time, std::vector<simulation_api_schema::TrafficLight>> new_prediction{predicted_time, traffic_light_states};
      predictions_for_id.emplace_back(new_prediction);
    }
  }

auto TrafficLights::isAnyTrafficLightChanged() -> bool
{
  return conventional_traffic_lights_->isAnyTrafficLightChanged() or
         v2i_traffic_lights_->isAnyTrafficLightChanged();
}

auto TrafficLights::startTrafficLightsUpdate(
  const double conventional_traffic_light_update_rate, const double v2i_traffic_lights_update_rate)
  -> void
{
  conventional_traffic_lights_->startUpdate(conventional_traffic_light_update_rate);
  v2i_traffic_lights_->startUpdate(v2i_traffic_lights_update_rate);
}

auto TrafficLights::getConventionalTrafficLights() const
  -> std::shared_ptr<ConventionalTrafficLights>
{
  return conventional_traffic_lights_;
}

auto TrafficLights::getV2ITrafficLights() const -> std::shared_ptr<V2ITrafficLights>
{
  return v2i_traffic_lights_;
}
}  // namespace traffic_simulator
