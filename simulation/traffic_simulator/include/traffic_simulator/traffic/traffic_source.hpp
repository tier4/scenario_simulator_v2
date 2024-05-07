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
  using VehicleParameter = traffic_simulator_msgs::msg::VehicleParameters;

  using PedestrianParameter = traffic_simulator_msgs::msg::PedestrianParameters;

  using VehicleOrPedestrianParameter = std::variant<VehicleParameter, PedestrianParameter>;

  struct Validator
  {
    static constexpr std::size_t spawning_lanes_limit = 1000;

    const lanelet::Ids ids;

    const lanelet::Lanelets lanelets;

    explicit Validator(
      const std::shared_ptr<hdmap_utils::HdMapUtils> &, const geometry_msgs::msg::Pose &,
      const double source_radius, const bool include_crosswalk);

    /**
     * @brief whether the 2D polygon does fit inside the lanelet with the given id
     */
    auto operator()(const std::vector<geometry_msgs::msg::Point> &, lanelet::Id) const -> bool;
  };

  struct Configuration
  {
    bool allow_spawn_outside_lane = false;

    bool require_footprint_fitting = false;

    bool use_random_orientation = false;

    double start_delay = 0.0;
  };

  using Distribution =
    std::vector<std::tuple<VehicleOrPedestrianParameter, std::string, std::string, double>>;

  template <typename Pose, typename Parameters>
  using Spawner = std::function<void(
    const std::string &, const Pose &, const Parameters &, const std::string &,
    const std::string &)>;

  template <typename Spawner>
  explicit TrafficSource(
    const double radius, const double rate, const geometry_msgs::msg::Pose & pose,
    const Distribution & distribution, const std::optional<int> seed, const double current_time,
    const Configuration & configuration,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils, const Spawner & spawn)
  : rate(rate),
    pose(pose),
    id(source_count_++),
    spawn_vehicle_in_lane_coordinate(spawn),
    spawn_pedestrian_in_lane_coordinate(spawn),
    spawn_vehicle_in_world_coordinate(spawn),
    spawn_pedestrian_in_world_coordinate(spawn),
    hdmap_utils_(hdmap_utils),
    engine_(seed ? seed.value() : std::random_device()()),
    angle_distribution_(0.0, boost::math::constants::two_pi<double>()),
    radius_distribution_(0.0, radius),
    params_distribution_([&]() {
      auto weights = std::vector<double>();
      weights.reserve(distribution.size());
      for ([[maybe_unused]] auto && [parameter, behavior, model3d, weight] : distribution) {
        weights.push_back(weight);
      }
      return std::discrete_distribution<std::size_t>(weights.begin(), weights.end());
    }()),
    start_execution_time_(
      /// @note Failsafe for the case where TrafficSource is added before the simulation starts or delay is negative
      (std::isnan(current_time) ? 0.0 : current_time) + std::max(configuration.start_delay, 0.0)),
    configuration_(configuration),
    distribution_(distribution),
    validate(hdmap_utils, pose, radius_distribution_.max(), false),
    validate_considering_crosswalk(hdmap_utils, pose, radius_distribution_.max(), true)
  {
  }

  void execute(const double current_time, const double step_time) override;

  const double rate;

  const geometry_msgs::msg::Pose pose;

  const std::size_t id;

private:
  auto makeRandomPose(const bool random_orientation = false) -> geometry_msgs::msg::Pose;

  auto isPoseValid(const VehicleOrPedestrianParameter &, const geometry_msgs::msg::Pose &)
    -> std::pair<bool, std::optional<CanonicalizedLaneletPose>>;

  static inline std::size_t source_count_ = 0;

  std::size_t entity_count_ = 0;

  const Spawner<CanonicalizedLaneletPose, VehicleParameter> spawn_vehicle_in_lane_coordinate;

  const Spawner<CanonicalizedLaneletPose, PedestrianParameter> spawn_pedestrian_in_lane_coordinate;

  const Spawner<geometry_msgs::msg::Pose, VehicleParameter> spawn_vehicle_in_world_coordinate;

  const Spawner<geometry_msgs::msg::Pose, PedestrianParameter> spawn_pedestrian_in_world_coordinate;

  const std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_;

  std::mt19937 engine_;

  std::uniform_real_distribution<double> angle_distribution_;

  std::uniform_real_distribution<double> radius_distribution_;

  std::discrete_distribution<std::size_t> params_distribution_;

  const double start_execution_time_;

  const Configuration configuration_;

  const Distribution distribution_;

  /// @note Validators, one does not allow positions on crosswalk, the other does allow positions on crosswalk
  const Validator validate, validate_considering_crosswalk;
};
}  // namespace traffic
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__TRAFFIC__TRAFFIC_SOURCE_HPP_
