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
class TrafficSink : public TrafficModuleBase
{
public:
  /**
   * @brief Construct a new Traffic Sink object
   * @param entity_manager_ptr Shared pointer, refers to the EntityManager
   * @param radius Radius of the sink
   * @param position Position of the traffic sink.
   * @param sinkable_entity_type Candidates for despawn. If empty, all entities are sinkable
   */
  explicit TrafficSink(
    const std::shared_ptr<entity::EntityManager> entity_manager_ptr, const double radius,
    const geometry_msgs::msg::Point & position,
    const std::set<std::uint8_t> & sinkable_entity_type);
  void execute(const double current_time, const double step_time) override;
  auto appendDebugMarker(visualization_msgs::msg::MarkerArray & marker_array) const
    -> void override;

  const std::size_t unique_id;
  const double radius;
  const geometry_msgs::msg::Point position;

private:
  auto getEntityNames() const -> std::vector<std::string>;
  auto getEntityType(const std::string & entity_name) const noexcept(false) -> EntityType;
  auto getEntityPose(const std::string & entity_name) const noexcept(false)
    -> geometry_msgs::msg::Pose;
  /** 
   *  @note Despawn the entity only when both:
   *  1. Its distance from the TrafficSink is <= radius [m].
   *  2. Its EntityType is in sinkable_entity_type or sinkable_entity_type is empty.
   */
  auto despawn(const std::string & entity_name) const -> void;

  const std::shared_ptr<entity::EntityManager> entity_manager_ptr;
  const std::set<std::uint8_t> sinkable_entity_type;
  inline static std::size_t unique_id_counter = 0UL;
};
}  // namespace traffic
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__TRAFFIC__TRAFFIC_SINK_HPP_
