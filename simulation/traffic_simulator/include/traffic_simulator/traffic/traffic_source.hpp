/**
 * @file traffic_source.hpp
 * @author Mateusz Palczuk (mateusz.palczuk@robotec.ai)
 * @brief class definition of the traffic source
 * @version 0.1
 * @date 2024-03-14
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

#ifndef TRAFFIC_SIMULATOR__TRAFFIC__TRAFFIC_SOURCE_HPP_
#define TRAFFIC_SIMULATOR__TRAFFIC__TRAFFIC_SOURCE_HPP_

#include <functional>
#include <geometry_msgs/msg/pose.hpp>
#include <map>
#include <random>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/traffic/traffic_module_base.hpp>

namespace traffic_simulator
{
namespace traffic
{
class TrafficSource : public TrafficModuleBase
{
public:
  explicit TrafficSource(
    const double radius, const double rate, const double speed,
    const geometry_msgs::msg::Point & position, std::optional<int> random_seed,
    const std::function<void(const std::string &, const geometry_msgs::msg::Pose &, const double)> &
      spawn_function,
    std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils);
  const double radius;
  const double rate;
  const double speed;
  const geometry_msgs::msg::Point position;
  void execute(const double current_time, const double step_time) override;

private:
  auto getSmallestSValue(const lanelet::Id id) -> double;
  auto getBiggestSValue(const lanelet::Id id) -> double;
  auto convertToPoseInArea(const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose)
    -> std::optional<geometry_msgs::msg::Pose>;
  auto getNewEntityName() -> std::string;

  auto getValidRandomPose() -> geometry_msgs::msg::Pose;
  auto getRandomLaneletPose() -> traffic_simulator_msgs::msg::LaneletPose;
  auto getRandomLaneletId() -> lanelet::Id;
  auto getRandomSValue(const lanelet::Id lanelet_id) -> double;

  inline static unsigned int next_source_id = 0u;
  const unsigned int source_id;
  const std::function<void(const std::string &, const geometry_msgs::msg::Pose &, const double)>
    spawn_function;
  const std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils;
  lanelet::Ids spawnable_lanelets;
  std::mt19937 engine;
  std::uniform_int_distribution<std::size_t> id_distribution;
  std::map<std::size_t, std::uniform_real_distribution<double>> s_distributions;
  unsigned int entity_id = 0u;
  double last_spawn_time = 0.0;
};
}  // namespace traffic
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__TRAFFIC__TRAFFIC_SOURCE_HPP_
