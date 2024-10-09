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

#include <cpp_mock_scenarios/cpp_scenario_node.hpp>
#include <iostream>

namespace cpp_mock_scenarios
{
CppScenarioNode::CppScenarioNode(
  const std::string & node_name, const std::string & map_path,
  const std::string & lanelet2_map_file, const std::string & scenario_filename, const bool verbose,
  const rclcpp::NodeOptions & option)
: Node(node_name, option),
  api_(
    this, configure(map_path, lanelet2_map_file, scenario_filename, verbose),
    declare_parameter<double>("global_real_time_factor", 1.0),
    declare_parameter<double>("global_frame_rate", 20.0)),
  scenario_filename_(scenario_filename),
  exception_expect_(false)
{
  declare_parameter<std::string>("junit_path", "/tmp");
  get_parameter<std::string>("junit_path", junit_path_);
  declare_parameter<int>("global_timeout", 10.0);
  get_parameter<int>("global_timeout", timeout_);

  traffic_simulator::lanelet_pose::CanonicalizedLaneletPose::setConsiderPoseByRoadSlope([&]() {
    if (not has_parameter("consider_pose_by_road_slope")) {
      declare_parameter("consider_pose_by_road_slope", false);
    }
    return get_parameter("consider_pose_by_road_slope").as_bool();
  }());
}

void CppScenarioNode::update()
{
  onUpdate();
  try {
    api_.updateFrame();
    if (api_.getCurrentTime() >= timeout_) {
      stop(Result::FAILURE);
    }
  } catch (const common::scenario_simulator_exception::Error & e) {
    RCLCPP_ERROR_STREAM(get_logger(), e.what());
    if (exception_expect_) {
      stop(Result::SUCCESS);
    } else {
      stop(Result::FAILURE);
    }
  }
}

void CppScenarioNode::start()
{
  onInitialize();
  api_.startNpcLogic();
  const auto rate =
    std::chrono::duration<double>(1.0 / get_parameter("global_frame_rate").as_double());
  update_timer_ = this->create_wall_timer(rate, std::bind(&CppScenarioNode::update, this));
}

void CppScenarioNode::stop(Result result, const std::string & description)
{
  junit_.testsuite("cpp_mock_scenario");
  switch (result) {
    case Result::SUCCESS: {
      common::junit::Pass pass_case;
      junit_.testsuite("cpp_mock_scenario").testcase(scenario_filename_).pass.push_back(pass_case);
      std::cout << "cpp_scenario:success" << std::endl;
      break;
    }
    case Result::FAILURE: {
      common::junit::Failure failure_case("result", "failure");
      failure_case.message = description;
      junit_.testsuite("cpp_mock_scenario")
        .testcase(scenario_filename_)
        .failure.push_back(failure_case);
      std::cerr << "cpp_scenario:failure" << std::endl;
      break;
    }
  }
  // junit_.testsuite("cpp_mock_scenario").testcase(scenario_filename_).time = api_.getCurrentTime();
  junit_.write_to(junit_path_.c_str(), "  ");
  update_timer_->cancel();
  rclcpp::shutdown();
  std::exit(0);
}

void CppScenarioNode::spawnEgoEntity(
  const traffic_simulator::CanonicalizedLaneletPose & spawn_lanelet_pose,
  const std::vector<traffic_simulator::CanonicalizedLaneletPose> & goal_lanelet_poses,
  const traffic_simulator_msgs::msg::VehicleParameters & parameters)
{
  api_.updateFrame();
  std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / 20.0));
  api_.spawn("ego", spawn_lanelet_pose, parameters, traffic_simulator::VehicleBehavior::autoware());
  api_.asFieldOperatorApplication("ego").declare_parameter<bool>("allow_goal_modification", true);
  api_.attachLidarSensor("ego", 0.0);

  api_.attachDetectionSensor("ego", 200.0, true, 0.0, 0, 0.0, 0.0);

  api_.attachOccupancyGridSensor([this] {
    simulation_api_schema::OccupancyGridSensorConfiguration configuration;
    // clang-format off
      configuration.set_architecture_type(api_.getROS2Parameter<std::string>("architecture_type", "awf/universe"));
      configuration.set_entity("ego");
      configuration.set_filter_by_range(true);
      configuration.set_height(200);
      configuration.set_range(300);
      configuration.set_resolution(0.5);
      configuration.set_update_duration(0.1);
      configuration.set_width(200);
    // clang-format on
    return configuration;
  }());
  api_.requestAssignRoute("ego", goal_lanelet_poses);

  using namespace std::chrono_literals;
  while (!api_.asFieldOperatorApplication("ego").engaged()) {
    if (api_.asFieldOperatorApplication("ego").engageable()) {
      api_.asFieldOperatorApplication("ego").engage();
    }
    api_.updateFrame();
    std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / 20.0));
  }
}

auto CppScenarioNode::isVehicle(const std::string & name) const -> bool
{
  return api_.getEntity(name)->getEntityType().type ==
         traffic_simulator_msgs::msg::EntityType::VEHICLE;
}

auto CppScenarioNode::isPedestrian(const std::string & name) const -> bool
{
  return api_.getEntity(name)->getEntityType().type ==
         traffic_simulator_msgs::msg::EntityType::PEDESTRIAN;
}

void CppScenarioNode::checkConfiguration(const traffic_simulator::Configuration & configuration)
{
  try {
    configuration.getLanelet2MapFile();
    configuration.getPointCloudMapFile();
  } catch (const common::SimulationError &) {
    stop(Result::FAILURE);
  }
}
}  // namespace cpp_mock_scenarios
