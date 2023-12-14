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
//
// Co-developed by TIER IV, Inc. and Robotec.AI sp. z o.o.

#include "random_test_runner/test_randomizer.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>

#include "traffic_simulator/helper/helper.hpp"

static constexpr int max_randomization_attempts = 100;
static constexpr double min_npc_distance = 5.0;

const std::string & pedestrianPlannerBehaviorFromString(const std::string & input)
{
  if (input == "context_gamma_planner")
  {
    return traffic_simulator::PedestrianBehavior::contextGamma();
  }
  if (input == "do_nothing")
  {
    return traffic_simulator::PedestrianBehavior::doNothing();
  }
  if (input == "default")
  {
    return traffic_simulator::PedestrianBehavior::defaultBehavior();
  }
  if (input == "behavior_tree")
  {
    return traffic_simulator::PedestrianBehavior::behaviorTree();
  }

  throw std::runtime_error(fmt::format("Unknown pedestrian planner behavior: {}", input));
}

TestRandomizer::TestRandomizer(
  rclcpp::Logger logger, const TestSuiteParameters & test_suite_parameters,
  const TestCaseParameters & test_case_parameters, std::shared_ptr<LaneletUtils> lanelet_utils)
: logger_(logger),
  lanelet_utils_(std::move(lanelet_utils)),
  lanelet_ids_(lanelet_utils_->getLaneletIds()),
  randomization_engine_(std::make_shared<RandomizationEngine>(test_case_parameters.seed)),
  lanelet_id_randomizer_(randomization_engine_, 0, static_cast<int>(lanelet_ids_.size()) - 1),
  lanelet_offset_randomizer_(randomization_engine_,
    test_suite_parameters.npc_pedestrian_lanelet_min_offset, test_suite_parameters.npc_pedestrian_lanelet_max_offset),
  s_value_randomizer_(randomization_engine_, 0.0, 1.0),
  vehicle_npc_speed_randomizer_(
    randomization_engine_, test_suite_parameters.npc_vehicle_min_speed,
    test_suite_parameters.npc_vehicle_max_speed),
  pedestrian_npc_speed_randomizer_(randomization_engine_,
    test_suite_parameters.npc_pedestrian_min_speed,
    test_suite_parameters.npc_pedestrian_max_speed),
  test_suite_parameters_(test_suite_parameters)
{
  if (lanelet_ids_.empty()) {
    throw std::runtime_error("Lanelet ids vector is empty");
  }
}

TestDescription TestRandomizer::generate()
{
  TestDescription ret;

  std::tie(ret.ego_start_position, ret.ego_goal_position) = generateEgoRoute(
    test_suite_parameters_.ego_goal_lanelet_id, test_suite_parameters_.ego_goal_s,
    test_suite_parameters_.ego_goal_partial_randomization,
    test_suite_parameters_.ego_goal_partial_randomization_distance);
  ret.ego_goal_pose = lanelet_utils_->toMapPose(ret.ego_goal_position).pose;

  std::vector<LaneletPart> lanelets_around_start = lanelet_utils_->getLanesWithinDistance(
    ret.ego_start_position, test_suite_parameters_.npc_vehicle_min_spawn_distance_from_ego,
    test_suite_parameters_.npc_vehicle_max_spawn_distance_from_ego);

  std::vector<traffic_simulator_msgs::msg::LaneletPose> npc_vehicle_poses;
  for (int npc_id = 0; npc_id < test_suite_parameters_.npc_vehicle_count; npc_id++) {
    ret.npcs_vehicle_descriptions.emplace_back(generateVehicleNpcFromLaneletsWithMinDistanceFromPoses(
      npc_id, npc_vehicle_poses, min_npc_distance, lanelets_around_start));
    npc_vehicle_poses.emplace_back(ret.npcs_vehicle_descriptions.back().start_position);
  }

  std::vector<PedestrianBehavior> pedestrian_behaviors = pedestrianBehaviorsFromTestSuiteParameters(test_suite_parameters_);
  std::vector<traffic_simulator_msgs::msg::LaneletPose> crosswalk_poses = generateCrosswalkStartAndEndPoses();

  std::vector<traffic_simulator_msgs::msg::LaneletPose> npc_pedestrian_poses;
  for (int npc_id = 0; npc_id < test_suite_parameters_.npc_pedestrian_count; npc_id++) {
    ret.npcs_pedestrian_descriptions.emplace_back(generatePedestrianNpcFromLaneletsWithMinDistanceFromPoses(
      npc_id, npc_pedestrian_poses, pedestrian_behaviors, min_npc_distance, lanelets_around_start, crosswalk_poses));
    npc_pedestrian_poses.emplace_back(ret.npcs_pedestrian_descriptions.back().spawn_position);
  }

  return ret;
}

std::pair<traffic_simulator_msgs::msg::LaneletPose, traffic_simulator_msgs::msg::LaneletPose>
TestRandomizer::generateEgoRoute(
  int64_t goal_lanelet_id, double goal_s, bool partial_randomization, double randomization_distance)
{
  traffic_simulator_msgs::msg::LaneletPose goal_pose;
  auto goal_pose_from_params =
    traffic_simulator::helper::constructLaneletPose(goal_lanelet_id, goal_s);
  for (int attempt_number = 0; attempt_number < max_randomization_attempts; attempt_number++) {
    if (goal_lanelet_id < 0) {
      RCLCPP_INFO(logger_, "Goal randomization: full");
      goal_pose = generateRandomPosition();
    } else if (partial_randomization) {
      std::string message =
        fmt::format("Goal randomization: partial within distance: {}", randomization_distance);
      RCLCPP_INFO_STREAM(logger_, message);
      std::vector<LaneletPart> lanelets_around_goal =
        lanelet_utils_->getLanesWithinDistance(goal_pose_from_params, 0.0, randomization_distance);
      goal_pose = generatePoseFromLanelets(lanelets_around_goal);
    } else {
      RCLCPP_INFO(logger_, "Goal randomization: none - taken directly from parameters");
      goal_pose = goal_pose_from_params;
    }

    auto start_pose = generateRandomPosition();

    if (isFeasibleRoute(start_pose, goal_pose)) {
      return {start_pose, goal_pose};
    }
  }
  throw std::runtime_error("Was not able to randomize ego path - are boundaries too tight?");
}

traffic_simulator_msgs::msg::LaneletPose
TestRandomizer::generateRandomPoseWithinMinDistanceFromPosesFromLanelets(
  const std::vector<traffic_simulator_msgs::msg::LaneletPose> & poses, double min_distance,
  const std::vector<LaneletPart> & lanelets, double offset)
{
  for (int attempt_number = 0; attempt_number < max_randomization_attempts; attempt_number++) {
    auto ret = generatePoseFromLanelets(lanelets, offset);
    if (poses.empty()) {
      return ret;
    }
    double current_min_distance = std::numeric_limits<double>::max();
    for (const auto & pose : poses) {
      double distance = lanelet_utils_->computeDistance(pose, ret);
      current_min_distance = std::min(distance, current_min_distance);
    }
    if (current_min_distance > min_distance) {
      return ret;
    }
  }
  throw std::runtime_error(
    "Was not able to randomize pose within distance - "
    "are boundaries too tight?");
}

bool TestRandomizer::isFeasibleRoute(
  const traffic_simulator_msgs::msg::LaneletPose & start,
  const traffic_simulator_msgs::msg::LaneletPose & goal)
{
  if (start.lanelet_id == goal.lanelet_id && start.s < goal.s) {
    return true;
  }

  auto opposite_lanelet = lanelet_utils_->getOppositeLaneLet(goal);
  return !lanelet_utils_->getRoute(start.lanelet_id, goal.lanelet_id).empty() ||
         (opposite_lanelet &&
          !lanelet_utils_->getRoute(start.lanelet_id, opposite_lanelet->lanelet_id).empty());
}

int64_t TestRandomizer::getRandomLaneletId()
{
  return lanelet_ids_[lanelet_id_randomizer_.generate()];
}

double TestRandomizer::getRandomS(int64_t lanelet_id)
{
  return lanelet_utils_->getLaneletLength(lanelet_id) * s_value_randomizer_.generate();
}

double TestRandomizer::getRandomS(const LaneletPart & lanelet)
{
  return lanelet.start_s + (lanelet.end_s - lanelet.start_s) * s_value_randomizer_.generate();
}

traffic_simulator_msgs::msg::LaneletPose TestRandomizer::generateRandomPosition()
{
  const int64_t lanelet_id = getRandomLaneletId();
  return traffic_simulator::helper::constructLaneletPose(lanelet_id, getRandomS(lanelet_id));
}

std::vector<traffic_simulator_msgs::msg::LaneletPose> TestRandomizer::generateCrosswalkStartAndEndPoses()
{
  auto crosswalk_ids = lanelet_utils_->filterLaneletIds(lanelet_ids_, lanelet::AttributeValueString::Crosswalk);
  std::vector<traffic_simulator_msgs::msg::LaneletPose> crosswalk_poses;
  for (const auto & crosswalk_id : crosswalk_ids)
  {
    crosswalk_poses.emplace_back(traffic_simulator::helper::constructLaneletPose(crosswalk_id, 0.0));
    //HACK(kielczykowski-rai): when point is set on full lanelet length, ss2 throws that LaneletPose is invalid
    // when trying to approach goal
    crosswalk_poses.emplace_back(traffic_simulator::helper::constructLaneletPose(
      crosswalk_id, lanelet_utils_->getLaneletLength(crosswalk_id) - 1e-6));
  }
  return crosswalk_poses;
}

traffic_simulator_msgs::msg::LaneletPose TestRandomizer::generatePoseFromLanelets(
  const std::vector<LaneletPart> & lanelets, double offset)
{
  if (lanelets.empty()) {
    throw std::runtime_error("Lanelets from which position will be randomized cannot be empty");
  }
  LaneletIdRandomizer npc_lanelet_id_randomizer(randomization_engine_, 0, lanelets.size() - 1);
  LaneletPart lanelet_part = lanelets[npc_lanelet_id_randomizer.generate()];
  return traffic_simulator::helper::constructLaneletPose(
    lanelet_part.lanelet_id, getRandomS(lanelet_part), offset);
}

NPCVehicleDescription TestRandomizer::generateVehicleNpcFromLaneletsWithMinDistanceFromPoses(
  int npc_id, const std::vector<traffic_simulator_msgs::msg::LaneletPose> & poses,
  double min_distance, const std::vector<LaneletPart> & lanelets)
{
  std::stringstream npc_name_ss;
  npc_name_ss << "vehicleNPC" << npc_id;
  return {
    generateRandomPoseWithinMinDistanceFromPosesFromLanelets(poses, min_distance, lanelets),
    vehicle_npc_speed_randomizer_.generate(), npc_name_ss.str()};
}

NPCPedestrianDescription TestRandomizer::generatePedestrianNpcFromLaneletsWithMinDistanceFromPoses(
  int npc_id, const std::vector<traffic_simulator_msgs::msg::LaneletPose> & poses,
  const std::vector<PedestrianBehavior> & pedestrian_behaviors,
  double min_distance, const std::vector<LaneletPart> & lanelets,
  const std::vector<traffic_simulator_msgs::msg::LaneletPose> & crosswalk_poses)
{
  std::stringstream npc_name_ss;
  npc_name_ss << "pedestrianNPC" << npc_id;
  PedestrianBehavior behavior = getRandomPedestrianBehavior(pedestrian_behaviors);

  double speed = pedestrian_npc_speed_randomizer_.generate();
  if (behavior == PedestrianBehavior::STATIC)
  {
    speed = 0.0;
  }

  traffic_simulator_msgs::msg::LaneletPose spawn_pose;
  std::vector<geometry_msgs::msg::Pose> route;
  std::tie(spawn_pose, route) = generatePedestrianSpawnPointAndRouteFromBehavior(
    behavior, poses, min_distance, lanelets, crosswalk_poses);

  return {
    npc_name_ss.str(),
    speed,
    pedestrianPlannerBehaviorFromString(test_suite_parameters_.npc_pedestrian_planner),
    behavior,
    spawn_pose,
    route
  };
}

std::pair<traffic_simulator_msgs::msg::LaneletPose, geometry_msgs::msg::Pose> TestRandomizer::getRandomCrosswalkStartAndEndPose(
  const std::vector<traffic_simulator_msgs::msg::LaneletPose> & crosswalk_poses)
{
  UniformRandomizer<int64_t> crosswalk_id_randomizer = UniformRandomizer<int64_t>(randomization_engine_, 0, (crosswalk_poses.size() / 2) - 1);
  UniformRandomizer<int64_t> direction_randomizer = UniformRandomizer<int64_t>(randomization_engine_, 0, 1);
  UniformRandomizer<double> start_position_randomizer = UniformRandomizer<double>(randomization_engine_, -0.5, 0.5);

  int64_t index = crosswalk_id_randomizer.generate();
  auto start_lanelet_pose = crosswalk_poses[index * 2];
  auto goal_lanelet_pose = crosswalk_poses[index * 2 + 1];
  if (direction_randomizer.generate() == 1) {
    auto tmp_pose = start_lanelet_pose;
    start_lanelet_pose = goal_lanelet_pose;
    goal_lanelet_pose = tmp_pose;
  }
  start_lanelet_pose.offset = start_position_randomizer.generate();
  auto goal_pose_stamped = lanelet_utils_->toMapPose(goal_lanelet_pose);
  auto goal_pose = goal_pose_stamped.pose;
  return {start_lanelet_pose, goal_pose};
}

PedestrianBehavior TestRandomizer::getRandomPedestrianBehavior(const std::vector<PedestrianBehavior> & pedestrian_behaviors)
{
  UniformRandomizer<int64_t> pedestrian_behavior_id_randomizer = UniformRandomizer<int64_t>(randomization_engine_, 0, pedestrian_behaviors.size() - 1);
  return pedestrian_behaviors[pedestrian_behavior_id_randomizer.generate()];
}

std::pair<traffic_simulator_msgs::msg::LaneletPose, std::vector<geometry_msgs::msg::Pose>>
  TestRandomizer::generatePedestrianSpawnPointAndRouteFromBehavior(
    const PedestrianBehavior & behavior,
    const std::vector<traffic_simulator_msgs::msg::LaneletPose> & poses,
    double min_distance, const std::vector<LaneletPart> & lanelets,
    const std::vector<traffic_simulator_msgs::msg::LaneletPose> & crosswalk_poses)
{
  std::vector<geometry_msgs::msg::Pose> route;
  traffic_simulator_msgs::msg::LaneletPose spawn_lanelet_pose;
  switch(behavior)
  {
    case PedestrianBehavior::STATIC:
    {
      spawn_lanelet_pose = getRandomPedestrianSpawnPose(poses, min_distance, lanelets);
      auto lanelet_id = getRandomLaneletId();
      auto dummy_goal_pose_stamped = lanelet_utils_->toMapPose(
        traffic_simulator::helper::constructLaneletPose(lanelet_id, lanelet_utils_->getLaneletLength(lanelet_id)));
      route.emplace_back(dummy_goal_pose_stamped.pose);
      break;
    }
    case PedestrianBehavior::CROSSWALK:
    {
      geometry_msgs::msg::Pose goal_pose;
      std::tie(spawn_lanelet_pose, goal_pose) = getRandomCrosswalkStartAndEndPose(crosswalk_poses);
      route.emplace_back(goal_pose);
      break;
    }
    case PedestrianBehavior::WALK_ALONG_LANE:
      spawn_lanelet_pose = getRandomPedestrianSpawnPose(poses, min_distance, lanelets);
      auto positions = lanelet_utils_->getPositionsOnNextLanelets(spawn_lanelet_pose, 5, lanelet_offset_randomizer_.generate());
      route = positions;
      break;
  }
  return {spawn_lanelet_pose, route};
}

traffic_simulator_msgs::msg::LaneletPose TestRandomizer::getRandomPedestrianSpawnPose(
  const std::vector<traffic_simulator_msgs::msg::LaneletPose> & poses,
  double min_distance, const std::vector<LaneletPart> & lanelets)
{
  for (int attempt_number = 0; attempt_number < max_randomization_attempts; attempt_number++)
  {
    auto spawn_lanelet_pose = generateRandomPoseWithinMinDistanceFromPosesFromLanelets(poses, min_distance, lanelets, lanelet_offset_randomizer_.generate());
    auto spawn_pose = lanelet_utils_->toMapPose(spawn_lanelet_pose).pose;
    auto map_pose = lanelet_utils_->toLaneletPose(spawn_pose, false);
    if (!map_pose)
    {
      return spawn_lanelet_pose;
    }
  }

  throw std::runtime_error("Was not able to randomize NPC pedestrian spawn position.");
}
