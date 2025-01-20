/**
 * @file traffic_sink.hpp
 * @author Masaya Kataoka (masaya.kataoka@tier4.jp)
 * @brief class definition of the traffic sink
 * @version 0.1
 * @date 2021-04-01
 *
 * @copyright Copyright(c) TIER IV.Inc {2015}
 *
 */

// Copyright 2015 TIER IV.inc. All rights reserved.
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

#ifndef TRAFFIC_SIMULATOR__TRAFFIC__TRAFFIC_SINK_HPP_
#define TRAFFIC_SIMULATOR__TRAFFIC__TRAFFIC_SINK_HPP_

#include <lanelet2_core/geometry/Lanelet.h>

#include <functional>
#include <geometry_msgs/msg/pose.hpp>
#include <string>
#include <traffic_simulator/data_type/entity_status.hpp>
#include <traffic_simulator/entity/entity_manager.hpp>
#include <traffic_simulator/traffic/traffic_module_base.hpp>
#include <vector>

namespace traffic_simulator
{
namespace traffic
{
struct TrafficSinkConfig
{
  /**
   * @brief Construct a new TrafficSinkConfig object
   * @param radius Radius of the traffic sink
   * @param position Position of the traffic sink
   * @param sinkable_entity_types Candidates for despawn.
   */
  explicit TrafficSinkConfig(
    const double radius, const geometry_msgs::msg::Point & position,
    const std::set<std::uint8_t> & sinkable_entity_types,
    const std::optional<lanelet::Id> lanelet_id_opt)
  : radius(radius),
    position(position),
    sinkable_entity_types(sinkable_entity_types),
    description([](const std::optional<lanelet::Id> lanelet_id_opt) -> std::string {
      static long unique_id = 0L;
      if (lanelet_id_opt.has_value()) {
        return std::string("auto_") + std::to_string(lanelet_id_opt.value());
      } else {
        return std::string("custom_") + std::to_string(unique_id++);
      }
    }(lanelet_id_opt))
  {
  }

  const double radius;
  const geometry_msgs::msg::Point position;
  const std::set<std::uint8_t> sinkable_entity_types;
  const std::string description;
};

class TrafficSink : public TrafficModuleBase
{
public:
  /**
   * @brief Construct a new Traffic Sink object
   * @param entity_manager_ptr Shared pointer, refers to the EntityManager
   * @param config TrafficSink configuration
   */
  explicit TrafficSink(
    const std::function<void(const std::string &)> & despawn_function,
    const std::shared_ptr<entity::EntityManager> entity_manager_ptr,
    const TrafficSinkConfig & config);
  /** 
   *  @note execute calls despawn on each entity only when both:
   *  1. Its distance from the TrafficSink is <= config.radius [m].
   *  2. Its EntityType is in config.sinkable_entity_types.
   */
  auto execute(const double current_time, const double step_time) -> void override;
  auto appendDebugMarker(visualization_msgs::msg::MarkerArray & marker_array) const
    -> void override;

private:
  auto isEntitySinkable(const std::string & entity_name) const noexcept(false) -> bool;

  const std::function<void(const std::string &)> despawn_;
  const std::shared_ptr<entity::EntityManager> entity_manager_ptr_;
  const TrafficSinkConfig config_;
};
}  // namespace traffic
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__TRAFFIC__TRAFFIC_SINK_HPP_
