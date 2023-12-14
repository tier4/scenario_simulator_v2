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

#ifndef RANDOM_TEST_RUNNER__TESTRANDOMIZER_HPP
#define RANDOM_TEST_RUNNER__TESTRANDOMIZER_HPP

#include <optional>
#include <rclcpp/logger.hpp>

#include "random_test_runner/data_types.hpp"
#include "random_test_runner/lanelet_utils.hpp"
#include "random_test_runner/randomizers.hpp"
#include "traffic_simulator/api/api.hpp"

class LaneletUtils;

class TestRandomizer
{
public:
  TestRandomizer(
    rclcpp::Logger logger, const TestSuiteParameters & test_suite_parameters,
    const TestCaseParameters & test_case_parameters, std::shared_ptr<LaneletUtils> lanelet_utils);

  TestDescription generate();

private:
  bool isFeasibleRoute(
    const traffic_simulator_msgs::msg::LaneletPose & start,
    const traffic_simulator_msgs::msg::LaneletPose & goal);
  traffic_simulator_msgs::msg::LaneletPose generateRandomPoseWithinMinDistanceFromPosesFromLanelets(
    const std::vector<traffic_simulator_msgs::msg::LaneletPose> & poses, double min_distance,
    const std::vector<LaneletPart> & lanelets, double offset = 0.0);
  std::pair<traffic_simulator_msgs::msg::LaneletPose, traffic_simulator_msgs::msg::LaneletPose>
  generateEgoRoute(
    int64_t goal_lanelet_id, double goal_s, bool partial_randomization,
    double randomization_distance);
  int64_t getRandomLaneletId();
  double getRandomS(int64_t lanelet_id);
  double getRandomS(const LaneletPart & lanelet);
  traffic_simulator_msgs::msg::LaneletPose generateRandomPosition();
  traffic_simulator_msgs::msg::LaneletPose generatePoseFromLanelets(
    const std::vector<LaneletPart> & lanelets, double offset = 0.0);
  std::vector<traffic_simulator_msgs::msg::LaneletPose> generateCrosswalkStartAndEndPoses();
  NPCVehicleDescription generateVehicleNpcFromLaneletsWithMinDistanceFromPoses(
    int npc_id, const std::vector<traffic_simulator_msgs::msg::LaneletPose> & poses,
    double min_distance, const std::vector<LaneletPart> & lanelets);
  NPCPedestrianDescription generatePedestrianNpcFromLaneletsWithMinDistanceFromPoses(
    int npc_id, const std::vector<traffic_simulator_msgs::msg::LaneletPose> & poses,
    const std::vector<PedestrianBehavior> & pedestrian_behaviors,
    double min_distance, const std::vector<LaneletPart> & lanelets,
  const std::vector<traffic_simulator_msgs::msg::LaneletPose> & crosswalk_poses);
  std::pair<traffic_simulator_msgs::msg::LaneletPose, geometry_msgs::msg::Pose> getRandomCrosswalkStartAndEndPose(
    const std::vector<traffic_simulator_msgs::msg::LaneletPose> & crosswalk_poses);
  PedestrianBehavior getRandomPedestrianBehavior(
    const std::vector<PedestrianBehavior> & pedestrian_behaviors);
  std::pair<traffic_simulator_msgs::msg::LaneletPose, std::vector<geometry_msgs::msg::Pose>>
    generatePedestrianSpawnPointAndRouteFromBehavior(
      const PedestrianBehavior & behavior,
      const std::vector<traffic_simulator_msgs::msg::LaneletPose> & poses,
      double min_distance, const std::vector<LaneletPart> & lanelets,
    const std::vector<traffic_simulator_msgs::msg::LaneletPose> & crosswalk_poses);
  traffic_simulator_msgs::msg::LaneletPose getRandomPedestrianSpawnPose(
    const std::vector<traffic_simulator_msgs::msg::LaneletPose> & poses,
    double min_distance, const std::vector<LaneletPart> & lanelets);

  rclcpp::Logger logger_;

  std::shared_ptr<LaneletUtils> lanelet_utils_;
  std::vector<int64_t> lanelet_ids_;

  RandomizationEnginePtr randomization_engine_;
  LaneletIdRandomizer lanelet_id_randomizer_;
  LaneletOffsetRandomizer lanelet_offset_randomizer_;
  SValueRandomizer s_value_randomizer_;
  SpeedRandomizer vehicle_npc_speed_randomizer_;
  SpeedRandomizer pedestrian_npc_speed_randomizer_;

  TestSuiteParameters test_suite_parameters_;
};

#endif  // RANDOM_TEST_RUNNER__TESTRANDOMIZER_HPP
