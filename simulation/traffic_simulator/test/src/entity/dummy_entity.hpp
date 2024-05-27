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

#ifndef TRAFFIC_SIMULATOR__TEST__ENTITY__DUMMY_ENTITY_HPP_
#define TRAFFIC_SIMULATOR__TEST__ENTITY__DUMMY_ENTITY_HPP_

#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/entity/entity_base.hpp>
#include <traffic_simulator/helper/helper.hpp>

class DummyEntity : public traffic_simulator::entity::EntityBase
{
public:
  explicit DummyEntity(
    const std::string & name, const traffic_simulator::CanonicalizedEntityStatus & entity_status,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  : EntityBase(name, entity_status, hdmap_utils_ptr)
  {
  }

  auto getBehaviorParameter() const -> traffic_simulator_msgs::msg::BehaviorParameter override
  {
    return behavior_parameter_;
  }

  auto setBehaviorParameter(const traffic_simulator_msgs::msg::BehaviorParameter & params)
    -> void override
  {
    behavior_parameter_ = params;
  }

  auto getCurrentAction() const -> std::string override { return {}; }

  auto getDefaultDynamicConstraints() const
    -> const traffic_simulator_msgs::msg::DynamicConstraints & override
  {
    static const auto default_dynamic_constraints = []() {
      auto dynamic_constraints = traffic_simulator_msgs::msg::DynamicConstraints();
      dynamic_constraints.max_speed = 3.0;
      dynamic_constraints.max_acceleration = 5.0;
      dynamic_constraints.max_acceleration_rate = 7.0;
      dynamic_constraints.max_deceleration = 11.0;
      dynamic_constraints.max_deceleration_rate = 13.0;
      return dynamic_constraints;
    }();

    return default_dynamic_constraints;
  }

  auto getEntityType() const -> const traffic_simulator_msgs::msg::EntityType & override
  {
    return entity_type_;
  }

  auto getEntityTypename() const -> const std::string & override
  {
    static const auto str = std::string("dummy_entity");
    return str;
  }

  ~DummyEntity() override = default;

  auto getGoalPoses() -> std::vector<traffic_simulator::CanonicalizedLaneletPose> override
  {
    return {};
  }

  auto getObstacle() -> std::optional<traffic_simulator_msgs::msg::Obstacle> override
  {
    return std::nullopt;
  }

  auto getRouteLanelets(double) -> lanelet::Ids override { return route_lanelet_ids_; }

  auto getWaypoints() -> const traffic_simulator_msgs::msg::WaypointsArray override
  {
    return traffic_simulator_msgs::msg::WaypointsArray();
  }

  // clang-format off
  auto fillLaneletPose(traffic_simulator::CanonicalizedEntityStatus &) -> void override {}
  auto setVelocityLimit(double) -> void override {}
  auto setAccelerationLimit(double) -> void override {}
  auto setAccelerationRateLimit(double) -> void override {}
  auto setDecelerationLimit(double) -> void override {}
  auto setDecelerationRateLimit(double) -> void override {}
  auto requestAssignRoute(const std::vector<traffic_simulator::CanonicalizedLaneletPose> &) -> void override {}
  auto requestAssignRoute(const std::vector<geometry_msgs::msg::Pose> &) -> void override {}
  auto requestAcquirePosition(const traffic_simulator::CanonicalizedLaneletPose &) -> void override {}
  auto requestAcquirePosition(const geometry_msgs::msg::Pose &) -> void override {}
  // clang-format on

  /**
   * Function overriden so that other `requestLaneChange` functions can be tested.
   */

  auto requestLaneChange(const traffic_simulator::lane_change::Parameter & param) -> void override
  {
    lane_change_param_ = param;
  }

  /**
   * Additional fields and functions used in tests.
   */

  auto setRouteLanelets(const lanelet::Ids & ids) -> void { route_lanelet_ids_ = ids; }

  auto setEntityType(uint8_t value) -> void { entity_type_.type = value; }

  auto appendToJobList(
    const std::function<bool(const double)> & func_on_update,
    const std::function<void()> & func_on_cleanup, traffic_simulator::job::Type type,
    bool exclusive, const traffic_simulator::job::Event event) -> void
  {
    job_list_.append(func_on_update, func_on_cleanup, type, exclusive, event);
  }

  auto getTargetSpeed() -> std::optional<double> { return target_speed_; }

  auto getLaneChangeParameter() const -> traffic_simulator::lane_change::Parameter
  {
    return lane_change_param_;
  }

  auto getOtherStatus() const
    -> const std::unordered_map<std::string, traffic_simulator::CanonicalizedEntityStatus> &
  {
    return other_status_;
  }

  traffic_simulator_msgs::msg::BehaviorParameter behavior_parameter_;
  traffic_simulator_msgs::msg::EntityType entity_type_;
  traffic_simulator::lane_change::Parameter lane_change_param_;
  lanelet::Ids route_lanelet_ids_;
};

#endif  // TRAFFIC_SIMULATOR__TEST__ENTITY__DUMMY_ENTITY_HPP_
