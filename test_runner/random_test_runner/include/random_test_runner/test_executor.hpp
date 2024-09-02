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

#ifndef RANDOM_TEST_RUNNER__TEST_EXECUTOR_HPP
#define RANDOM_TEST_RUNNER__TEST_EXECUTOR_HPP

#include <memory>
#include <rclcpp/logger.hpp>

#include "random_test_runner/data_types.hpp"
#include "random_test_runner/file_interactions/junit_xml_reporter.hpp"
#include "random_test_runner/metrics/almost_standstill_metric.hpp"
#include "random_test_runner/metrics/ego_collision_metric.hpp"
#include "random_test_runner/metrics/goal_reached_metric.hpp"
#include "traffic_simulator/api/api.hpp"

traffic_simulator_msgs::msg::VehicleParameters getVehicleParameters()
{
  traffic_simulator_msgs::msg::VehicleParameters parameters;
  parameters.name = "vehicle.volkswagen.t";
  parameters.subtype.value = traffic_simulator_msgs::msg::EntitySubtype::CAR;
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

template <typename traffic_simulator_api_type>
class TestExecutor
{
public:
  TestExecutor(
    std::shared_ptr<traffic_simulator_api_type> api, TestDescription description,
    JunitXmlReporterTestCase test_case_reporter, double test_timeout,
    ArchitectureType architecture_type, rclcpp::Logger logger)
  : api_(std::move(api)),
    test_description_(std::move(description)),
    error_reporter_(std::move(test_case_reporter)),
    test_timeout_(test_timeout),
    architecture_type_(architecture_type),
    logger_(logger)
  {
  }

  void initialize()
  {
    executeWithErrorHandling([this]() {
      std::string message = fmt::format("Test description: {}", test_description_);
      RCLCPP_INFO_STREAM(logger_, message);
      scenario_completed_ = false;

      if (const auto ego_start_canonicalized_lanelet_pose = traffic_simulator::pose::canonicalize(
            test_description_.ego_start_position, api_->getHdmapUtils());
          !ego_start_canonicalized_lanelet_pose) {
        throw std::runtime_error(
          "Can not canonicalize ego start lanelet pose: id: " +
          std::to_string(test_description_.ego_start_position.lanelet_id) +
          " s: " + std::to_string(test_description_.ego_start_position.s) +
          " offset: " + std::to_string(test_description_.ego_start_position.offset));
      } else {
        api_->updateFrame();
        api_->spawn(
          ego_name_, ego_start_canonicalized_lanelet_pose.value(), getVehicleParameters(),
          traffic_simulator::VehicleBehavior::autoware(), "lexus_rx450h");
        api_->setEntityStatus(
          ego_name_, ego_start_canonicalized_lanelet_pose.value(),
          traffic_simulator::helper::constructActionStatus());

        if (architecture_type_ == ArchitectureType::AWF_UNIVERSE) {
          api_->attachLidarSensor(traffic_simulator::helper::constructLidarConfiguration(
            traffic_simulator::helper::LidarType::VLP16, ego_name_,
            stringFromArchitectureType(architecture_type_)));

          double constexpr detection_update_duration = 0.1;
          api_->attachDetectionSensor(
            traffic_simulator::helper::constructDetectionSensorConfiguration(
              ego_name_, stringFromArchitectureType(architecture_type_),
              detection_update_duration));

          api_->attachOccupancyGridSensor([&]() {
            simulation_api_schema::OccupancyGridSensorConfiguration configuration;
            configuration.set_architecture_type(stringFromArchitectureType(architecture_type_));
            configuration.set_entity(ego_name_);
            configuration.set_filter_by_range(true);
            configuration.set_height(200);
            configuration.set_range(300);
            configuration.set_resolution(0.5);
            configuration.set_update_duration(0.1);
            configuration.set_width(200);
            return configuration;
          }());

          api_->asFieldOperatorApplication(ego_name_).template declare_parameter<bool>(
            "allow_goal_modification", true);
        }
      }

      // XXX dirty hack: wait for autoware system to launch
      // ugly but helps for now
      std::this_thread::sleep_for(std::chrono::milliseconds{5000});

      api_->requestAssignRoute(ego_name_, std::vector({test_description_.ego_goal_pose}));
      api_->asFieldOperatorApplication(ego_name_).engage();

      goal_reached_metric_.setGoal(test_description_.ego_goal_pose);

      for (size_t i = 0; i < test_description_.npcs_descriptions.size(); i++) {
        const auto & npc_descr = test_description_.npcs_descriptions[i];
        if (const auto npc_start_canonicalized_lanelet_pose = traffic_simulator::pose::canonicalize(
              npc_descr.start_position, api_->getHdmapUtils());
            !npc_start_canonicalized_lanelet_pose) {
          throw std::runtime_error(
            "Can not canonicalize npc start lanelet pose: id: " +
            std::to_string(npc_descr.start_position.lanelet_id) +
            " s: " + std::to_string(npc_descr.start_position.s) +
            " offset: " + std::to_string(npc_descr.start_position.offset));
        } else {
          api_->spawn(
            npc_descr.name, npc_start_canonicalized_lanelet_pose.value(), getVehicleParameters(),
            traffic_simulator::VehicleBehavior::defaultBehavior(), "taxi");
          api_->setEntityStatus(
            npc_descr.name, npc_start_canonicalized_lanelet_pose.value(),
            traffic_simulator::helper::constructActionStatus(npc_descr.speed));
          api_->requestSpeedChange(npc_descr.name, npc_descr.speed, true);
        }
      }
    });
  }

  auto update() -> void
  {
    executeWithErrorHandling([this]() {
      if (not api_->isEgoSpawned() and not api_->isNpcLogicStarted()) {
        api_->startNpcLogic();
      }
      if (
        api_->isEgoSpawned() and not api_->isNpcLogicStarted() and
        api_->asFieldOperatorApplication(api_->getEgoName()).engageable()) {
        api_->startNpcLogic();
      }

      auto current_time = api_->getCurrentTime();

      if (!std::isnan(current_time)) {
        if (goal_reached_metric_.isGoalReached(api_->getEntityStatus(ego_name_))) {
          scenario_completed_ = true;
        }

        bool timeout_reached = current_time >= test_timeout_;
        if (timeout_reached) {
          if (!goal_reached_metric_.isGoalReached(api_->getEntityStatus(ego_name_))) {
            RCLCPP_INFO(logger_, "Timeout reached");
            error_reporter_.reportTimeout();
          }
          scenario_completed_ = true;
          return;
        }
      }

      for (const auto & npc : test_description_.npcs_descriptions) {
        if (api_->entityExists(npc.name) && api_->checkCollision(ego_name_, npc.name)) {
          if (ego_collision_metric_.isThereEgosCollisionWith(npc.name, current_time)) {
            std::string message =
              fmt::format("New collision occurred between ego and {}", npc.name);
            RCLCPP_INFO_STREAM(logger_, message);
            error_reporter_.reportCollision(npc, current_time);
          }
        }
      }

      if (almost_standstill_metric_.isAlmostStandingStill(api_->getEntityStatus(ego_name_))) {
        RCLCPP_INFO(logger_, "Standstill duration exceeded");
        if (goal_reached_metric_.isGoalReached(api_->getEntityStatus(ego_name_))) {
          RCLCPP_INFO(logger_, "Goal reached, standstill expected");
        } else {
          error_reporter_.reportStandStill();
        }
        scenario_completed_ = true;
      }

      api_->updateFrame();
    });
  }

  void deinitialize()
  {
    executeWithErrorHandling([this]() {
      std::string message = fmt::format("Deinitialize: {}", test_description_);
      RCLCPP_INFO_STREAM(logger_, message);

      api_->despawn(ego_name_);

      for (const auto & npc : test_description_.npcs_descriptions) {
        api_->despawn(npc.name);
      }
    });
  }

  bool scenarioCompleted() { return scenario_completed_; }

private:
  void executeWithErrorHandling(std::function<void()> && func)
  {
    try {
      func();
    } catch (const common::AutowareError & error) {
      error_reporter_.reportException("autoware error", error.what());
      std::string message = fmt::format("common::AutowareError occurred: {}", error.what());
      RCLCPP_ERROR_STREAM(logger_, message);
      scenario_completed_ = true;
    } catch (const common::scenario_simulator_exception::Error & error) {
      error_reporter_.reportException("scenario simulator error", error.what());
      std::string message =
        fmt::format("common::scenario_simulator_exception::Error occurred: {}", error.what());
      RCLCPP_ERROR_STREAM(logger_, message);
      scenario_completed_ = true;
    } catch (const std::runtime_error & error) {
      std::string message = fmt::format("std::runtime_error occurred: {}", error.what());
      RCLCPP_ERROR_STREAM(logger_, message);
      scenario_completed_ = true;
    } catch (...) {
      RCLCPP_ERROR(logger_, "Unknown error occurred.");
      scenario_completed_ = true;
    }
  }

  std::shared_ptr<traffic_simulator_api_type> api_;
  TestDescription test_description_;
  const std::string ego_name_ = "ego";

  AlmostStandstillMetric almost_standstill_metric_;
  GoalReachedMetric goal_reached_metric_;
  EgoCollisionMetric ego_collision_metric_;
  JunitXmlReporterTestCase error_reporter_;

  double test_timeout_;
  ArchitectureType architecture_type_;

  bool scenario_completed_ = false;

  rclcpp::Logger logger_;
};

#endif  // RANDOM_TEST_RUNNER__TEST_EXECUTOR_HPP
