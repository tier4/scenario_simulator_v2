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

#ifndef TRAFFIC_SIMULATOR__TRAFFIC__TRAFFIC_SOURCE_HPP_
#define TRAFFIC_SIMULATOR__TRAFFIC__TRAFFIC_SOURCE_HPP_

#include <functional>
#include <geometry_msgs/msg/pose.hpp>
#include <map>
#include <random>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/traffic/traffic_module_base.hpp>
#include <traffic_simulator_msgs/msg/pedestrian_parameters.hpp>
#include <traffic_simulator_msgs/msg/vehicle_parameters.hpp>

namespace traffic_simulator
{
namespace traffic
{
class TrafficSource : public TrafficModuleBase
{
public:
  using VehicleParams = traffic_simulator_msgs::msg::VehicleParameters;
  using PedestrianParams = traffic_simulator_msgs::msg::PedestrianParameters;
  struct Configuration
  {
    Configuration() = default;
    Configuration(const bool allow_spawn_outside_lane, const bool require_footprint_fitting)
    : allow_spawn_outside_lane(allow_spawn_outside_lane),
      require_footprint_fitting(require_footprint_fitting){};
    bool allow_spawn_outside_lane = false;
    bool require_footprint_fitting = false;
    bool use_random_orientation = false;
    double start_time = 0.0;
  };
  explicit TrafficSource(
    const double radius, const double rate, const double speed,
    const geometry_msgs::msg::Pose & position,
    const std::vector<std::pair<std::variant<VehicleParams, PedestrianParams>, double>> & params,
    const std::optional<int> random_seed,
    const std::function<void(
      const std::string &, const geometry_msgs::msg::Pose &, const VehicleParams &, const double)> &
      vehicle_spawn_function,
    const std::function<void(
      const std::string &, const geometry_msgs::msg::Pose &, const PedestrianParams &,
      const double)> & pedestrian_spawn_function,
    const Configuration & configuration, std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils);
  const double radius_;
  const double rate_;
  const double speed_;
  const geometry_msgs::msg::Pose source_pose_;
  void execute(const double current_time, const double step_time) override;

private:
  auto getRandomPose(const bool random_orientation = false) -> geometry_msgs::msg::Pose;
  auto getSmallestSValue(const lanelet::Id id) -> double;
  auto getBiggestSValue(const lanelet::Id id) -> double;
  auto convertToPoseInArea(const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose)
    -> std::optional<geometry_msgs::msg::Pose>;
  auto getNewEntityName() -> std::string;
  auto isPedestrian(const std::variant<VehicleParams, PedestrianParams> & params) -> bool;

  auto isPoseValid(const geometry_msgs::msg::Pose & pose) -> bool;
  auto getValidRandomPose() -> geometry_msgs::msg::Pose;
  auto getRandomLaneletId() -> lanelet::Id;
  auto getRandomSValue(const lanelet::Id lanelet_id) -> double;
  auto randomizeParams() -> void;

  inline static unsigned int next_source_id_ = 0u;
  const unsigned int source_id_;
  const std::function<void(
    const std::string &, const geometry_msgs::msg::Pose &, const VehicleParams &, const double)>
    vehicle_spawn_function_;
  const std::function<void(
    const std::string &, const geometry_msgs::msg::Pose &, const PedestrianParams &, const double)>
    pedestrian_spawn_function_;

  const std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_;
  lanelet::Ids spawnable_lanelets_;
  std::mt19937 engine_;
  std::uniform_int_distribution<std::size_t> id_distribution_;
  std::map<std::size_t, std::uniform_real_distribution<double>> s_distributions_;
  std::uniform_real_distribution<double> angle_distribution_;
  std::uniform_real_distribution<double> radius_distribution_;
  std::discrete_distribution<> params_distribution_;
  unsigned int entity_id_ = 0u;
  const double start_execution_time_ = 0.0;
  unsigned int spawn_count_ = 0u;
  const Configuration config_;
  std::vector<std::variant<VehicleParams, PedestrianParams>> params_;
  std::vector<std::variant<VehicleParams, PedestrianParams>>::iterator current_params_;
};
}  // namespace traffic
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__TRAFFIC__TRAFFIC_SOURCE_HPP_
