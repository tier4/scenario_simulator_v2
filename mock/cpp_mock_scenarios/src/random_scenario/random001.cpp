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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cpp_mock_scenarios/catalogs.hpp>
#include <cpp_mock_scenarios/cpp_scenario_node.hpp>
#include <memory>
#include <random001_parameters.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <traffic_simulator/api/api.hpp>
#include <traffic_simulator/utils/lanelet_map.hpp>
#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>
#include <vector>

class RandomScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit RandomScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "lanechange_left", ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map",
      "lanelet2_map.osm", __FILE__, false, option),
    param_listener_(std::make_shared<random001::ParamListener>(get_node_parameters_interface())),
    engine_(seed_gen_())
  {
    start();
  }

private:
  std::shared_ptr<random001::ParamListener> param_listener_;
  random001::Params params_;
  std::random_device seed_gen_;
  std::mt19937 engine_;
  double lane_change_position = 0.0;
  bool lane_change_requested = false;
  void spawnRoadParkingVehicles()
  {
    std::normal_distribution<> normal_dist(
      0.0, params_.random_parameters.road_parking_vehicle.s_variance);
    const auto spawn_road_parking_vehicle =
      [&](const auto & entity_index, const auto offset, const auto number_of_vehicles) {
        const auto get_entity_subtype = [](const std::string & entity_type) {
          if (entity_type == "car") {
            return traffic_simulator_msgs::msg::EntitySubtype::CAR;
          } else if (entity_type == "truck") {
            return traffic_simulator_msgs::msg::EntitySubtype::TRUCK;
          } else if (entity_type == "bus") {
            return traffic_simulator_msgs::msg::EntitySubtype::BUS;
          } else if (entity_type == "trailer") {
            return traffic_simulator_msgs::msg::EntitySubtype::TRAILER;
          }
          return traffic_simulator_msgs::msg::EntitySubtype::CAR;
        };

        std::string entity_name = "road_parking_" + std::to_string(entity_index);
        constexpr lanelet::Id spawn_lanelet_id = 34705;
        api_.spawn(
          entity_name,
          traffic_simulator::helper::constructCanonicalizedLaneletPose(
            spawn_lanelet_id,
            static_cast<double>(entity_index) / static_cast<double>(number_of_vehicles) *
                traffic_simulator::lanelet_map::laneletLength(spawn_lanelet_id) +
              normal_dist(engine_),
            offset),
          getVehicleParameters(
            get_entity_subtype(params_.random_parameters.road_parking_vehicle.entity_type)));
        api_.getEntity(entity_name).requestSpeedChange(0, true);
      };
    std::uniform_real_distribution<> dist(
      params_.random_parameters.road_parking_vehicle.min_offset,
      params_.random_parameters.road_parking_vehicle.max_offset);
    for (int i = 0; i < params_.random_parameters.road_parking_vehicle.number_of_vehicle; i++) {
      spawn_road_parking_vehicle(
        i, dist(engine_), params_.random_parameters.road_parking_vehicle.number_of_vehicle);
    }
  }

  /// Despawn parking entity before replacing parking entity.
  void despawnRoadParkingVehicles()
  {
    for (int i = 0; i < params_.random_parameters.road_parking_vehicle.number_of_vehicle; i++) {
      api_.despawn("road_parking_" + std::to_string(i));
    }
  }

  void despawnCrossingPedestrians()
  {
    for (int i = 0; i < params_.random_parameters.crossing_pedestrian.number_of_pedestrian; i++) {
      std::string entity_name = "pedestrian" + std::to_string(i);
      if (api_.isEntityExist(entity_name)) {
        api_.despawn(entity_name);
      }
    }
  }

  void onUpdate() override
  {
    const auto & ego_entity = api_.getEntity("ego");
    {
      if (param_listener_->is_old(params_)) {
        /// When the parameter was updated, clear entity before re-spawning entity.
        despawnRoadParkingVehicles();
        despawnCrossingPedestrians();
        param_listener_->refresh_dynamic_parameters();
        params_ = param_listener_->get_params();
        /// Re-spawn road parking vehicle.
        spawnRoadParkingVehicles();
      }
    };

    const auto spawn_and_change_lane = [&](const auto & entity_name, const auto spawn_s_value) {
      if (!api_.isEntityExist(entity_name)) {
        api_.spawn(
          entity_name,
          traffic_simulator::helper::constructCanonicalizedLaneletPose(34513, spawn_s_value, 0.0),
          getVehicleParameters());
        std::uniform_real_distribution<> speed_distribution(
          params_.random_parameters.lane_following_vehicle.min_speed,
          params_.random_parameters.lane_following_vehicle.max_speed);
        const auto speed = speed_distribution(engine_);
        auto & entity = api_.getEntity(entity_name);
        entity.requestSpeedChange(speed, true);
        entity.setLinearVelocity(speed);
        std::uniform_real_distribution<> lane_change_position_distribution(
          0.0, traffic_simulator::lanelet_map::laneletLength(34684));
        lane_change_position = lane_change_position_distribution(engine_);
        lane_change_requested = false;
      }
      /// Checking the ego entity overs the lane change position.
      if (ego_entity.isInLanelet()) {
        const auto lanelet_pose = ego_entity.getCanonicalizedStatus().getLaneletPose();
        if (lanelet_pose.lanelet_id == 34684 && std::abs(lanelet_pose.s) >= lane_change_position) {
          api_.getEntity(entity_name)
            .requestLaneChange(traffic_simulator::lane_change::Direction::RIGHT);
          lane_change_requested = true;
        }
      }
    };

    if (ego_entity.isInLanelet(34684, 0.1)) {
      spawn_and_change_lane("lane_following_0", 0.0);
    }

    /// Spawn and cross pedestrian if it does not exist and ego entity does not exists on lane "34576"
    const auto spawn_and_cross_pedestrian = [&](const auto & entity_index) {
      std::string entity_name = "pedestrian" + std::to_string(entity_index);
      constexpr lanelet::Id lanelet_id = 34392;
      if (
        !api_.isEntityExist(entity_name) &&
        !ego_entity.isNearbyPosition(
          traffic_simulator::helper::constructCanonicalizedLaneletPose(34576, 25.0, 0.0), 5.0)) {
        std::normal_distribution<> offset_distribution(
          0.0, params_.random_parameters.crossing_pedestrian.offset_variance);
        std::uniform_real_distribution<> speed_distribution(
          params_.random_parameters.crossing_pedestrian.min_speed,
          params_.random_parameters.crossing_pedestrian.max_speed);
        api_.spawn(
          entity_name,
          traffic_simulator::helper::constructCanonicalizedLaneletPose(
            lanelet_id, 0.0, offset_distribution(engine_)),
          getPedestrianParameters());
        const auto speed = speed_distribution(engine_);
        auto & entity = api_.getEntity(entity_name);
        entity.requestSpeedChange(speed, true);
        entity.setLinearVelocity(speed);
      }
      if (
        api_.isEntityExist(entity_name) &&
        api_.getEntity(entity_name).getStandStillDuration() >= 0.5) {
        api_.despawn(entity_name);
      }
    };
    for (int i = 0; i < params_.random_parameters.crossing_pedestrian.number_of_pedestrian; i++) {
      spawn_and_cross_pedestrian(i);
    }

    const auto trigger_position =
      traffic_simulator::helper::constructCanonicalizedLaneletPose(34621, 10.0, 0.0);
    const auto ego_goal_position =
      traffic_simulator::helper::constructCanonicalizedLaneletPose(34606, 0.0, 0.0);
    constexpr auto entity_name = "spawn_nearby_ego";

    if (ego_entity.isNearbyPosition(trigger_position, 20.0) && !api_.isEntityExist(entity_name)) {
      api_.spawn(
        entity_name,
        traffic_simulator::pose::transformRelativePoseToGlobal(
          ego_entity.getMapPose(),
          geometry_msgs::build<geometry_msgs::msg::Pose>()
            .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(10.0).y(-5.0).z(0.0))
            .orientation(geometry_msgs::msg::Quaternion())),
        getVehicleParameters(),
        traffic_simulator::entity::VehicleEntity::BuiltinBehavior::doNothing());
    }

    if (!ego_entity.isNearbyPosition(trigger_position, 20.0) && api_.isEntityExist(entity_name)) {
      api_.despawn(entity_name);
    }

    if (ego_entity.isNearbyPosition(ego_goal_position, 1.0)) {
      api_.despawn("ego");
      stop(cpp_mock_scenarios::Result::SUCCESS);
    }
  }

  void onInitialize() override
  {
    // api_.setVerbose(true);
    params_ = param_listener_->get_params();

    /// Spawn road parking vehicle with initial parameters.
    spawnRoadParkingVehicles();

    spawnEgoEntity(
      traffic_simulator::helper::constructCanonicalizedLaneletPose(34621, 10.0, 0.0),
      {traffic_simulator::helper::constructCanonicalizedLaneletPose(34606, 0.0, 0.0)},
      getVehicleParameters());
    if (api_.isEntityExist("ego")) {
      api_.spawn(
        "parking_outside",
        traffic_simulator::pose::transformRelativePoseToGlobal(
          api_.getEntity("ego").getMapPose(),
          geometry_msgs::build<geometry_msgs::msg::Pose>()
            .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(10).y(15).z(0))
            .orientation(geometry_msgs::msg::Quaternion())),
        getVehicleParameters(),
        traffic_simulator::entity::VehicleEntity::BuiltinBehavior::doNothing());
    } else {
      api_.despawn("ego");
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<RandomScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
