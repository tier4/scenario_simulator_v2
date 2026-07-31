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

TestRandomizer::TestRandomizer(
  rclcpp::Logger logger, const TestSuiteParameters & test_suite_parameters,
  const TestCaseParameters & test_case_parameters, std::shared_ptr<LaneletUtils> lanelet_utils)
: logger_(logger),
  lanelet_utils_(std::move(lanelet_utils)),
  lanelet_ids_(lanelet_utils_->getLaneletIds()),
  randomization_engine_(std::make_shared<RandomizationEngine>(test_case_parameters.seed)),
  lanelet_id_randomizer_(randomization_engine_, 0, static_cast<int>(lanelet_ids_.size()) - 1),
  s_value_randomizer_(randomization_engine_, 0.0, 1.0),
  speed_randomizer_(
    randomization_engine_, test_suite_parameters.npc_min_speed,
    test_suite_parameters.npc_max_speed),
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
  ret.ego_goal_pose = lanelet_utils_->toMapPose(ret.ego_goal_position, false).pose;

  std::vector<LaneletPart> lanelets_around_start = lanelet_utils_->getLanesWithinDistance(
    ret.ego_start_position, test_suite_parameters_.npc_min_spawn_distance_from_ego,
    test_suite_parameters_.npc_max_spawn_distance_from_ego);

  std::vector<traffic_simulator_msgs::msg::LaneletPose> npc_poses;
  for (int npc_id = 0; npc_id < test_suite_parameters_.npcs_count; npc_id++) {
    ret.npcs_descriptions.emplace_back(generateNpcFromLaneletsWithMinDistanceFromPoses(
      npc_id, npc_poses, min_npc_distance, lanelets_around_start));
    npc_poses.emplace_back(ret.npcs_descriptions.back().start_position);
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
  const std::vector<LaneletPart> & lanelets)
{
  for (int attempt_number = 0; attempt_number < max_randomization_attempts; attempt_number++) {
    auto ret = generatePoseFromLanelets(lanelets);
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

traffic_simulator_msgs::msg::LaneletPose TestRandomizer::generatePoseFromLanelets(
  const std::vector<LaneletPart> & lanelets)
{
  if (lanelets.empty()) {
    throw std::runtime_error("Lanelets from which position will be randomized cannot be empty");
  }
  LaneletIdRandomizer npc_lanelet_id_randomizer(randomization_engine_, 0, lanelets.size() - 1);
  LaneletPart lanelet_part = lanelets[npc_lanelet_id_randomizer.generate()];
  return traffic_simulator::helper::constructLaneletPose(
    lanelet_part.lanelet_id, getRandomS(lanelet_part));
}

NPCDescription TestRandomizer::generateNpcFromLaneletsWithMinDistanceFromPoses(
  int npc_id, const std::vector<traffic_simulator_msgs::msg::LaneletPose> & poses,
  double min_distance, const std::vector<LaneletPart> & lanelets)
{
  std::stringstream npc_name_ss;
  npc_name_ss << "npc" << npc_id;
  return {
    generateRandomPoseWithinMinDistanceFromPosesFromLanelets(poses, min_distance, lanelets),
    speed_randomizer_.generate(), npc_name_ss.str()};
}
