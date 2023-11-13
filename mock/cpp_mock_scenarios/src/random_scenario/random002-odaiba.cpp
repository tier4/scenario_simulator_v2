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
#include <random001_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traffic_simulator/api/api.hpp>

#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>
#include <traffic_simulator/helper/stop_watch.hpp>

#include <quaternion_operation/quaternion_operation.h>
#include "./random_util.hpp"

// headers in STL
#include <memory>
#include <random>
#include <string>
#include <vector>
#include <cstdlib>
#include <ctime>

class RandomScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit RandomScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "lanechange_left", /* ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map" */
      "/home/planning-control-developer/workspace/odaiba_beta",
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

  const size_t MAX_SPAWN_NUMBER = 10;
  bool ego_is_in_stuck_ = false;
  bool driving_to_destination_ = true;

  std::vector<lanelet::Id> goal_no1_candidate_ids_ = {190609, 179443, 190785, 190794};
  std::vector<lanelet::Id> goal_no2_candidate_ids_ = {300087};

  // 300087:テレポート駅路肩レーン
  // 179398:未来館駐車場走行レーン
  // 179443:未来館駐車場路肩レーン
  // 190785:未来館駐車場路肩レーン
  // 178481:未来館と走行レーンの間のレーン
  // 1290:未来館左折退場後右側走行レーン
  lanelet::Id start_lane_id_ = 300087;
  std::vector<lanelet::Id> route_to_destination_ids_ = {40, 179443};
  lanelet::Id destination_lane_id_ = 179443;
  std::vector<lanelet::Id> route_to_start_lane_ids_ = {190029, 1412, 300087};



  MyStopWatch<> sw_ego_stuck_;

  StateManager<std::string> tl_state_manager_{{"red", "amber", "green"}, {10.0, 3.0, 10.0}};
  StateManager<std::string> crosswalk_pedestrian_state_manager_{{"go", "stop"}, {15.0, 15.0}};

  void updateRoute(const std::vector<lanelet::Id> & new_goal_lane_ids)
  {
    std::vector<traffic_simulator::CanonicalizedLaneletPose> new_lane_poses;
    for (const auto & id : new_goal_lane_ids) {
      new_lane_poses.push_back(api_.canonicalize(constructLaneletPose(id, 5.0, 0, 0, 0, 0)));
    }
    api_.requestAssignRoute("ego", new_lane_poses);
  }

  // If ego is far from lane_id, remove all entities.
  // Return if the ego is close to the lane_id.
  bool removeFarNPCsAndCheckIsInTriggerDistance(
    const std::string & entity_name_prefix, const lanelet::Id & lane_id)
  {
    const auto removeEntities = [&]() {
      for (int i = 0; i < MAX_SPAWN_NUMBER; i++) {
        const std::string name = entity_name_prefix + "_" + std::to_string(i);
        if (api_.entityExists(name)) {
          api_.despawn(name);
        }
      }
    };

    constexpr auto untrigger_distance = 220.0;  // must be longer than trigger_distance
    constexpr auto trigger_distance = 200.0;  // must be shorter than untrigger_distance
    constexpr auto too_close_for_trigger_distance = 50.0;  // must be shorter than untrigger_distance
    const auto target_lane = api_.canonicalize(constructLaneletPose(lane_id, 0.0));

    const bool already_exist = api_.entityExists(entity_name_prefix + "_0");

    // Remove entities if the lane is far from ego.
    if (already_exist) {
      if (!api_.reachPosition("ego", target_lane, untrigger_distance)) {
        removeEntities();
      }
      return false;  // no need to spawn vehicles
    }

    // No need to spawn since the ego is too close to the lane.
    if (api_.reachPosition("ego", target_lane, too_close_for_trigger_distance)) {
      return false;  // no need to spawn vehicles
    }

    // No need to spawn since the ego is far from the lane.
    if (!api_.reachPosition("ego", target_lane, trigger_distance)) {
      return false;  // no need to spawn vehicles
    }

    return true;  // need to spawn vehicles
  }

  void spawnAndCrossPedestrian(
    const int entity_num_max, const lanelet::Id & spawn_lane_id, const lanelet::Id & goal_lane_id)
  {
    const std::string entity_name_prefix =
      "pedestrian_" + std::to_string(spawn_lane_id) + "_" + std::to_string(goal_lane_id);
    if (!removeFarNPCsAndCheckIsInTriggerDistance(entity_name_prefix, spawn_lane_id)) {
      return;
    }

    const auto & p = params_.random_parameters.crossing_pedestrian;
    std::normal_distribution<> offset_distribution(0.0, p.offset_variance);
    std::uniform_real_distribution<> speed_distribution(p.min_speed, p.max_speed);
    const auto spawn_pose = constructLaneletPose(spawn_lane_id, 0.0, offset_distribution(engine_));
    const auto goal_pose = constructLaneletPose(goal_lane_id, 5.0);

    // api_.requestWalkStraight();

    for (int entity_index = 0; entity_index < entity_num_max; ++entity_index) {
      const std::string entity_name = entity_name_prefix + "_" + std::to_string(entity_index);
      constexpr double reach_tolerance = 5.0;

      if (!api_.entityExists(entity_name)) {
        std::normal_distribution<> offset_distribution(0.0, p.offset_variance);
        std::uniform_real_distribution<> speed_distribution(p.min_speed, p.max_speed);
        api_.spawn(entity_name, api_.canonicalize(spawn_pose), getPedestrianParameters());
        if (crosswalk_pedestrian_state_manager_.getCurrentState() == "go") {
          const auto speed = speed_distribution(engine_);
          api_.requestSpeedChange(entity_name, speed, true);
          api_.setLinearVelocity(entity_name, speed);
        } else {
          api_.setLinearVelocity(entity_name, 0.0);
        }
      } else {
        if (
          crosswalk_pedestrian_state_manager_.getCurrentState() == "go" &&
          std::fabs(api_.getEntityStatus(entity_name).getTwist().linear.x) < 0.01) {
          const auto speed = speed_distribution(engine_);
          api_.requestSpeedChange(entity_name, speed, true);
          api_.setLinearVelocity(entity_name, speed);
        }
        if (api_.reachPosition("ego", api_.canonicalize(goal_pose), reach_tolerance)) {
          api_.despawn(entity_name);
        }
      }

    }
  }

  void spawnAndChangeLane(
    const std::string & entity_name, const LaneletPose & spawn_pose, const lanelet::Id & lane_change_id,
    const Direction & lane_change_direction)
  {
    const auto & p = params_.random_parameters.lane_following_vehicle;
    if (!api_.entityExists(entity_name)) {
      api_.spawn(entity_name, api_.canonicalize(spawn_pose), getVehicleParameters());
      std::uniform_real_distribution<> speed_distribution(p.min_speed, p.max_speed);
      const auto speed = speed_distribution(engine_);
      api_.requestSpeedChange(entity_name, speed, true);
      api_.setLinearVelocity(entity_name, speed);
      std::uniform_real_distribution<> lane_change_position_distribution(
        0.0, api_.getLaneletLength(lane_change_id));
      lane_change_position = lane_change_position_distribution(engine_);
      lane_change_requested = false;
    }
    const auto lanelet_pose = api_.getLaneletPose("ego");

    /// Checking the ego entity overs the lane change position.
    if (
      lanelet_pose && static_cast<LaneletPose>(lanelet_pose.value()).lanelet_id == lane_change_id &&
      std::abs(static_cast<LaneletPose>(lanelet_pose.value()).s) >= lane_change_position) {
      api_.requestLaneChange(entity_name, lane_change_direction);
      lane_change_requested = true;
    }
  }

  void spawnAndMoveToGoal(
    const lanelet::Id & spawn_lane_id, const lanelet::Id & goal_lane_id, const double min_v = 3.0,
    const double max_v = 18.0)
  {
    const std::string entity_name_prefix =
      "vehicle_move_to_goal_" + std::to_string(spawn_lane_id) + "_" + std::to_string(goal_lane_id);
    if (!removeFarNPCsAndCheckIsInTriggerDistance(entity_name_prefix, spawn_lane_id)) {
      return;
    }

    const auto spawn_pose = constructLaneletPose(spawn_lane_id, 0.0);
    const auto goal_pose = constructLaneletPose(goal_lane_id, 0.0);

    const auto entity_name = entity_name_prefix;
    if (!api_.entityExists(entity_name)) {
      api_.spawn(entity_name, api_.canonicalize(spawn_pose), getVehicleParameters());
      std::uniform_real_distribution<> speed_distribution(min_v, max_v);
      const auto speed = speed_distribution(engine_);
      api_.requestSpeedChange(entity_name, speed, true);
      api_.setLinearVelocity(entity_name, speed);
    }

    constexpr double reach_tolerance = 2.0;
    if (api_.reachPosition(entity_name, api_.canonicalize(goal_pose), reach_tolerance)) {
        api_.despawn(entity_name);
    }
  }


  void spawnRoadParkingVehicles(const lanelet::Id & spawn_lanelet_id, const size_t number_of_vehicles, const DIRECTION direction)
  {
    const std::string entity_name_prefix = "road_parking_" + std::to_string(spawn_lanelet_id);
    if (!removeFarNPCsAndCheckIsInTriggerDistance(entity_name_prefix, spawn_lanelet_id)) {
      return;
    }

    const auto & p = params_.random_parameters.road_parking_vehicle;
    std::normal_distribution<> normal_dist(0.0, p.s_variance);

    const auto object_type = [&]() {
      std::uniform_real_distribution<> dis(0.0, 1.0);
      double probability = dis(engine_);
      if (probability < 0.50) {
        return "car";  // 50% for "car"
      } else if (probability < 0.70) {
        return "truck";  // Additional 20% for "truck", total 70%
      } else if (probability < 0.90) {
        return "bus";  // Additional 20% for "bus", total 90%
      } else {
        return "trailer";  // Remaining 10% for "trailer"
      }
    }();

    std::cerr << "object type = " << object_type << std::endl;


    const auto spawn_road_parking_vehicle = [&](const auto & entity_index, const auto offset) {
      const std::string entity_name = entity_name_prefix + "_" + std::to_string(entity_index);
      const auto space = static_cast<double>(entity_index) / number_of_vehicles;
      const auto spawn_position =
        space * api_.getLaneletLength(spawn_lanelet_id) + normal_dist(engine_);
      const auto spawn_pose = constructLaneletPose(spawn_lanelet_id, spawn_position, offset, 0, 0);
      const auto vehicle_param = getVehicleParameters(get_entity_subtype(object_type));
      if (!api_.entityExists(entity_name)) {
        api_.spawn(entity_name, api_.canonicalize(spawn_pose), vehicle_param);
      }
      api_.requestSpeedChange(entity_name, 0, true);
    };

    const auto [min_offset, max_offset] = [&]() -> std::pair<double, double> {
      if (direction == DIRECTION::CENTER) {
        return {-0.5, 0.5};
      } else if (direction == DIRECTION::LEFT) {
        return {3.0, 1.0};
      } else if (direction == DIRECTION::RIGHT) {
        return {-1.0, -3.0};
      } else if (direction == DIRECTION::VERY_LEFT) {
        return {5.0, 3.0};
      } else if (direction == DIRECTION::VERY_RIGHT) {
        return {-3.0, -5.0};
      }
    }();


    std::uniform_real_distribution<> dist(min_offset, max_offset);
    for (size_t i = 0; i < number_of_vehicles; i++) {
      spawn_road_parking_vehicle(i, dist(engine_));
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
      if (api_.entityExists(entity_name)) {
        api_.despawn(entity_name);
      }
    }
  }

  void spawnAndDespawnRelativeFromEgoInRange(
    const lanelet::Id & trigger_lane_id, const double trigger_lane_s, const double trigger_range,
    const double rel_x, const double rel_y)
  {
    const auto trigger_position =
      api_.canonicalize(constructLaneletPose(trigger_lane_id, trigger_lane_s));
    const auto entity_name = "spawn_nearby_ego";
    if (
      api_.reachPosition("ego", trigger_position, trigger_range) &&
      !api_.entityExists(entity_name)) {
      api_.spawn(
        entity_name, api_.getMapPoseFromRelativePose("ego", createPose(rel_x, rel_y)),
        getVehicleParameters(),
        traffic_simulator::entity::VehicleEntity::BuiltinBehavior::doNothing());
    }
    if (
      !api_.reachPosition("ego", trigger_position, trigger_range) &&
      api_.entityExists(entity_name)) {
      api_.despawn(entity_name);
    }
  }

  // Set color for the traffic_right_id. Set opposite color (green <-> red) to the opposite_traffic_right_id
  void updateRandomTrafficLightColor(
    const std::vector<int> & traffic_right_ids, const std::vector<int> & opposite_traffic_right_ids,
    const std::string & tl_color)
  {
    const auto setTlColor = [&](const auto & ids, const std::string & color) {
      for (const auto id : ids) {
        for (traffic_simulator::TrafficLight & traffic_light :
             api_.getConventionalTrafficLights(id)) {
          traffic_light.clear();
          traffic_light.set(color + " solidOn circle");
        };
      }
    };

    setTlColor(traffic_right_ids, tl_color);
    setTlColor(opposite_traffic_right_ids, getOppositeTlColor(tl_color));
  }

  bool processForEgoStuck() {
    constexpr auto STUCK_TIME_THRESHOLD = 10.0;
    const auto stuck_time = api_.getStandStillDuration("ego");

    // 10秒以上スタックしたら、NPC全消ししてflag true、タイマースタート。
    if (stuck_time > STUCK_TIME_THRESHOLD && !ego_is_in_stuck_) {
      std::cerr << "\n\nEgo is in stuck. Remove all vehicles!!!\n\n" << std::endl;
      // api_.despawnEntities();
      auto entities = api_.getEntityNames();
      for (const auto & e : entities) {
        if (e != api_.getEgoName()) {
          api_.despawn(e);
        }
      }
      ego_is_in_stuck_ = true;
      sw_ego_stuck_.tic("ego_stuck");
    }

    // flagがtrueならstuck時の処理を確認。5秒経つまではNPC処理はしない。5秒立ったらフラグを下げる。
    // →　これは、egoの近傍過ぎるやつはspawnしないという処理を入れることで消せる。
    if (ego_is_in_stuck_) {
      if (sw_ego_stuck_.toc("ego_stuck") < 5.0) {
        return true;
      } else {
        ego_is_in_stuck_ = false;
        return false;
      }
    }
    return false;
  }

  void onUpdate() override
  {
#if 0    
    // パラメータが更新されたら、全車両を更新
    if (param_listener_->is_old(params_)) {
      despawnRoadParkingVehicles();
      despawnCrossingPedestrians();
      param_listener_->refresh_dynamic_parameters();
      params_ = param_listener_->get_params();

      /// Re-spawn road parking vehicle.
      spawnRoadParkingVehicles(34705);
    }

    // Spawn lane-changing vehicles
    if (api_.isInLanelet("ego", 34684, 0.1)) {
      spawnAndChangeLane(
        "lane_following_0", constructLaneletPose(34513, 0.0), 34684, Direction::RIGHT);
    }

    spawnAndDespawnRelativeFromEgoInRange(34621, 10.0, 20.0, 10.0, -5.0);
#endif

    constexpr double reach_tolerance = 10.0;
    const auto stuck_time = api_.getStandStillDuration("ego");
    const bool ego_is_in_stop = stuck_time > 5.0;
    if (ego_is_in_stop && driving_to_destination_) {
	std::cout << __func__ << ":" << __LINE__ << std::endl;
      const auto reach_target_lane =
        std::any_of(goal_no1_candidate_ids_.begin(), goal_no1_candidate_ids_.end(), [&, this](const auto & id){
    	  const auto target_lane = api_.canonicalize(constructLaneletPose(id, 5.0));
          return api_.reachPosition("ego", target_lane, reach_tolerance);
          // return api_.isInLanelet("ego", id, 5.0);
          });
      if(reach_target_lane) {
	std::cout << "REACH GOAL NO.1!!!" << std::endl;
        updateRoute(route_to_start_lane_ids_);
        driving_to_destination_ = false;
      }
    } else if (ego_is_in_stop && !driving_to_destination_) {
	std::cout << __func__ << ":" << __LINE__ << std::endl;
      const auto reach_target_lane =
        std::any_of(goal_no2_candidate_ids_.begin(), goal_no2_candidate_ids_.end(), [&, this](const auto & id){
    	  const auto target_lane = api_.canonicalize(constructLaneletPose(id, 5.0));
          return api_.reachPosition("ego", target_lane, reach_tolerance);
          // return api_.isInLanelet("ego", id, 5.0);
          });
      if (reach_target_lane) {
	std::cout << "REACH GOAL NO.2!!!" << std::endl;
        updateRoute(route_to_destination_ids_);
        driving_to_destination_ = true;
      }
    }

    if (processForEgoStuck()) {
      return;
    }

    constexpr double MIN_VEL = 5.0;
    constexpr double MAX_VEL = 20.0;

    // === 以下、egoが特定のレーンに近づいたらNPCを生成する ===

    // 静止物体を生成。位置はランダム、数もランダム。
    spawnRoadParkingVehicles(176671, randomInt(4, 4), DIRECTION::VERY_LEFT);  // unstable
    spawnRoadParkingVehicles(176640, randomInt(4, 4), DIRECTION::VERY_LEFT);  // unstable
    spawnRoadParkingVehicles(176148, randomInt(0, 4), DIRECTION::LEFT);  // unstable
    spawnRoadParkingVehicles(176193, randomInt(0, 4), DIRECTION::LEFT);
    // spawnRoadParkingVehicles(1501, randomInt(0, 4), DIRECTION::RIGHT);  // stuck多し
    spawnRoadParkingVehicles(1262, randomInt(1, 4), DIRECTION::RIGHT);  // 右駐車は避けれん
    spawnRoadParkingVehicles(1265, randomInt(1, 4), DIRECTION::LEFT);
    spawnRoadParkingVehicles(1278, randomInt(1, 2), DIRECTION::LEFT);
    // 未来館に静止物体生成
    spawnRoadParkingVehicles(179398, randomInt(1, 2), DIRECTION::LEFT);
    spawnRoadParkingVehicles(190784, randomInt(0, 1), DIRECTION::LEFT);
    spawnRoadParkingVehicles(190797, randomInt(0, 1), DIRECTION::LEFT);
    spawnRoadParkingVehicles(1513, randomInt(1, 2), DIRECTION::CENTER);
    spawnRoadParkingVehicles(1468, randomInt(1, 2), DIRECTION::CENTER);
    spawnRoadParkingVehicles(178766, randomInt(0, 1), DIRECTION::VERY_LEFT);
    spawnRoadParkingVehicles(179473, randomInt(0, 1), DIRECTION::VERY_LEFT);
    
    
    
    // 目的レーンまで動く移動物体を生成
    spawnAndMoveToGoal(176261, 176175, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(350, 163, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(350, 1506, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(1482, 38, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(1483, 38, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(1484, 39, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(1501, 40, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(32, 38, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(33, 39, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(34, 40, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(1314, 41, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(94, 41, MIN_VEL, MAX_VEL);
    
    spawnAndMoveToGoal(175378, 174994, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(1263, 106, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(1265, 178001, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(1153, 94, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(178233, 179475, MIN_VEL, MAX_VEL);

    spawnAndMoveToGoal(74, 84, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(75, 83, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(75, 178573, MIN_VEL, MAX_VEL);
    spawnAndMoveToGoal(1483, 1500, MIN_VEL, MAX_VEL);


    // 信号機の情報を変更
    updateRandomTrafficLightColor({10584}, {10589}, tl_state_manager_.getCurrentState());
    updateRandomTrafficLightColor({10324, 190343}, {10316, 10322}, tl_state_manager_.getCurrentState());
    updateRandomTrafficLightColor({10352}, {10356, 10359}, tl_state_manager_.getCurrentState());
    // updateRandomTrafficLightColor({179285, 10284}, {10293, 10283}, tl_state_manager_.getCurrentState());
    updateRandomTrafficLightColor({10269, 10276}, {10263, 10277}, tl_state_manager_.getCurrentState());
    updateRandomTrafficLightColor({10247, 10261}, {10249}, tl_state_manager_.getCurrentState());
    updateRandomTrafficLightColor({10237, 10236}, {10238}, tl_state_manager_.getCurrentState());
    updateRandomTrafficLightColor({10546, 10549}, {10551}, tl_state_manager_.getCurrentState());
    updateRandomTrafficLightColor({10562, 10564}, {10556}, tl_state_manager_.getCurrentState());
    updateRandomTrafficLightColor({10575, 10569}, {10571, 10581}, tl_state_manager_.getCurrentState());
    updateRandomTrafficLightColor({10610, 10598}, {10604}, tl_state_manager_.getCurrentState());
    updateRandomTrafficLightColor({10342}, {10343}, tl_state_manager_.getCurrentState());

    
    // 横断歩道歩行者をspawn（速度は毎回ランダム、人数はspawnで抽選、数秒ごとにspawnを止める）
    // spawnAndCrossPedestrian(3, 1561, 1561);
    // spawnAndCrossPedestrian(3, 1621, 1621);
    // spawnAndCrossPedestrian(2, 1606, 1606);
    // spawnAndCrossPedestrian(1, 1600, 1600);
    // spawnAndCrossPedestrian(2, 1599, 1599);
    // spawnAndCrossPedestrian(1, 1591, 1591);
    // spawnAndCrossPedestrian(1, 1586, 1586);
    // spawnAndCrossPedestrian(1, 1584, 1584);
    // spawnAndCrossPedestrian(2, 1625, 1625);
    // spawnAndCrossPedestrian(2, 1627, 1627);
    // spawnAndCrossPedestrian(2, 1628, 1628);
    // spawnAndCrossPedestrian(10, 1567, 1567);


    // NPCを出力させてLCする
    // if (api_.isInLanelet("ego", 34684, 0.1)) {
    //   spawnAndChangeLane(
    //     "lane_following_0", constructLaneletPose(34513, 0.0), 34684, Direction::RIGHT);
    // }

    // 自己位置の相対位置にnpcを配置
    // spawnAndDespawnRelativeFromEgoInRange(34621, 10.0, 20.0, 10.0, -5.0);

  }

  void onInitialize() override
  {
    // api_.setVerbose(true);

    srand(time(0));  // Initialize random seed

    params_ = param_listener_->get_params();

    // 初期値とゴールを設定。初期値は一度しか指定できない。ゴールは何度も指定できるが、Rvizから指定した方が早そう。
    // 路肩からの発進時に同じノード名で複数ノードが起動されてしまいautowareが発信できなくなってしまうエラーがあるため、duplicated_node_checkerを無効化する必要あり。
    const auto spawn_pose = api_.canonicalize(constructLaneletPose(start_lane_id_, 5.0, 0, 0, 0, 0));
    const auto goal_poses = [&](const std::vector<lanelet::Id> lane_ids) {
      std::vector<traffic_simulator::CanonicalizedLaneletPose> poses;
      for (const auto id : lane_ids) {
        poses.push_back(api_.canonicalize(constructLaneletPose(id, 5.0, 0, 0, 0, 0)));
      }
      return poses;
    }(route_to_destination_ids_);  // 最後がゴール、その前は並び順でcheck point
    spawnEgoEntity(spawn_pose, goal_poses, getVehicleParameters());

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
