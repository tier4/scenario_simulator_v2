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
#include <traffic_simulator/traffic/traffic_module_base.hpp>
#include <vector>

namespace traffic_simulator
{
namespace traffic
{
class TrafficSink : public TrafficModuleBase
{
public:
  explicit TrafficSink(
    lanelet::Id lanelet_id, double radius, const geometry_msgs::msg::Point & position,
    const std::function<std::vector<std::string>(void)> & get_entity_names_function,
    const std::function<geometry_msgs::msg::Pose(const std::string &)> & get_entity_pose_function,
    const std::function<void(std::string)> & despawn_function);
  const lanelet::Id lanelet_id;
  const double radius;
  const geometry_msgs::msg::Point position;
  void execute(const double current_time, const double step_time) override;
  auto appendDebugMarker(visualization_msgs::msg::MarkerArray & marker_array) const
    -> void override;

private:
  const std::function<std::vector<std::string>(void)> get_entity_names_function;
  const std::function<geometry_msgs::msg::Pose(const std::string &)> get_entity_pose_function;
  const std::function<void(const std::string &)> despawn_function;
};
}  // namespace traffic
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__TRAFFIC__TRAFFIC_SINK_HPP_
