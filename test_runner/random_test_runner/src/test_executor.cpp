// Copyright 2015-2021 Tier IV, Inc. All rights reserved.
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
// Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

#include "random_test_runner/test_executor.hpp"

#include <rclcpp/rclcpp.hpp>

#include "random_test_runner/file_interactions/junit_xml_reporter.hpp"
#include "random_test_runner/file_interactions/yaml_test_params_saver.hpp"

const double test_timeout = 60.0;
const bool attach_sensors = false;

traffic_simulator_msgs::msg::VehicleParameters getVehicleParameters()
{
  traffic_simulator_msgs::msg::VehicleParameters parameters;
  parameters.name = "vehicle.volkswagen.t";
  parameters.vehicle_category = "car";
  parameters.performance.max_speed = 69.444;
  parameters.performance.max_acceleration = 200;
  parameters.performance.max_deceleration = 10.0;
  parameters.bounding_box.center.x = 1.5;
  parameters.bounding_box.center.y = 0.0;
  parameters.bounding_box.center.z = 0.9;
  parameters.bounding_box.dimensions.x = 4.5;
  parameters.bounding_box.dimensions.y = 2.1;
  parameters.bounding_box.dimensions.z = 1.8;
  parameters.axles.front_axle.max_steering = 0.5;
  parameters.axles.front_axle.wheel_diameter = 0.6;
  parameters.axles.front_axle.track_width = 1.8;
  parameters.axles.front_axle.position_x = 3.1;
  parameters.axles.front_axle.position_z = 0.3;
  parameters.axles.rear_axle.max_steering = 0.0;
  parameters.axles.rear_axle.wheel_diameter = 0.6;
  parameters.axles.rear_axle.track_width = 1.8;
  parameters.axles.rear_axle.position_x = 0.0;
  parameters.axles.rear_axle.position_z = 0.3;
  return parameters;
}

TestExecutor::TestExecutor(
  std::shared_ptr<traffic_simulator::API> api, TestDescription description,
  JunitXmlReporterTestCase test_case_reporter, SimulatorType simulator_type, rclcpp::Logger logger)
: api_(std::move(api)),
  test_description_(std::move(description)),
  error_reporter_(std::move(test_case_reporter)),
  simulator_type_(simulator_type),
  logger_(logger)
{
}

void TestExecutor::initialize()
{
  RCLCPP_INFO(logger_, fmt::format("Test description: {}", test_description_).c_str());
  scenario_completed_ = false;

  api_->initialize(1.0, 0.05);
  api_->updateFrame();

  if (simulator_type_ == SimulatorType::SIMPLE_SENSOR_SIMULATOR) {
    api_->spawn(ego_name_, getVehicleParameters(), traffic_simulator::VehicleBehavior::autoware());
    api_->setEntityStatus(
      ego_name_, test_description_.ego_start_position,
      traffic_simulator::helper::constructActionStatus());

    if (attach_sensors) {
      api_->attachLidarSensor(traffic_simulator::helper::constructLidarConfiguration(
        traffic_simulator::helper::LidarType::VLP16, ego_name_, "/perception/points_nonground"));
    }

    // XXX dirty hack: wait for autoware system to launch, ugly but helps for
    // now
    std::this_thread::sleep_for(std::chrono::milliseconds{5000});

    api_->requestAssignRoute(
      ego_name_,
      std::vector<traffic_simulator_msgs::msg::LaneletPose>{test_description_.ego_goal_position});
    goal_reached_metric_.setGoal(test_description_.ego_goal_pose);
  }

  for (size_t i = 0; i < test_description_.npcs_descriptions.size(); i++) {
    const auto & npc_descr = test_description_.npcs_descriptions[i];
    api_->spawn(npc_descr.name, getVehicleParameters());
    api_->setEntityStatus(
      npc_descr.name, npc_descr.start_position,
      traffic_simulator::helper::constructActionStatus(npc_descr.speed));
    api_->setTargetSpeed(npc_descr.name, npc_descr.speed, true);
  }
}

void TestExecutor::update(double current_time)
{
  bool timeout_reached = current_time >= test_timeout;

  if (timeout_reached) {
    if (simulator_type_ == SimulatorType::SIMPLE_SENSOR_SIMULATOR) {
      traffic_simulator_msgs::msg::EntityStatus status = api_->getEntityStatus(ego_name_);
      if (!goal_reached_metric_.isGoalReached(status)) {
        RCLCPP_INFO(logger_, "Timeout reached");
        error_reporter_.reportTimeout();
      }
    }
    scenario_completed_ = true;
    return;
  }

  if (simulator_type_ == SimulatorType::SIMPLE_SENSOR_SIMULATOR) {
    traffic_simulator_msgs::msg::EntityStatus status = api_->getEntityStatus(ego_name_);
    for (const auto & npc : test_description_.npcs_descriptions) {
      if (api_->entityExists(npc.name) && api_->checkCollision(ego_name_, npc.name)) {
        if (ego_collision_metric_.isThereEgosCollisionWith(npc.name, current_time)) {
          RCLCPP_INFO(
            logger_, fmt::format("New collision occurred between ego and {}", npc.name).c_str());
          error_reporter_.reportCollision(npc, current_time);
        }
      }
    }

    if (almost_standstill_metric_.isAlmostStandingStill(status)) {
      RCLCPP_INFO(logger_, "Standstill duration exceeded");
      if (goal_reached_metric_.isGoalReached(status)) {
        RCLCPP_INFO(logger_, "Goal reached, standstill expected");
      } else {
        error_reporter_.reportStandStill();
      }
      scenario_completed_ = true;
    }
  }

  api_->updateFrame();
}

void TestExecutor::deinitialize()
{
  RCLCPP_INFO(logger_, fmt::format("Deinitialize: {}", test_description_).c_str());

  if (simulator_type_ == SimulatorType::SIMPLE_SENSOR_SIMULATOR) {
    api_->despawn(ego_name_);
  }
  for (const auto & npc : test_description_.npcs_descriptions) {
    api_->despawn(npc.name);
  }
}

bool TestExecutor::scenarioCompleted() { return scenario_completed_; }
