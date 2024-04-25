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
    return behavior_parameter;
  }

  void setBehaviorParameter(const traffic_simulator_msgs::msg::BehaviorParameter & params) override
  {
    behavior_parameter = params;
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
    return entity_type;
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

  std::optional<traffic_simulator_msgs::msg::Obstacle> getObstacle() override
  {
    return std::nullopt;
  }

  auto getRouteLanelets(double) -> lanelet::Ids override { return std::vector<lanelet::Id>{}; }

  auto getWaypoints() -> const traffic_simulator_msgs::msg::WaypointsArray override
  {
    return traffic_simulator_msgs::msg::WaypointsArray();
  }

  // clang-format off
  void fillLaneletPose(traffic_simulator::CanonicalizedEntityStatus &) override {}
  void setVelocityLimit(double) override {}
  void setAccelerationLimit(double) override {}
  void setAccelerationRateLimit(double) override {}
  void setDecelerationLimit(double) override {}
  void setDecelerationRateLimit(double) override {}
  void requestAssignRoute(const std::vector<traffic_simulator::CanonicalizedLaneletPose> &) override {}
  void requestAssignRoute(const std::vector<geometry_msgs::msg::Pose> &) override {}
  void requestAcquirePosition(const traffic_simulator::CanonicalizedLaneletPose &) override {}
  void requestAcquirePosition(const geometry_msgs::msg::Pose &) override {}
  // clang-format on

  traffic_simulator_msgs::msg::BehaviorParameter behavior_parameter;
  traffic_simulator_msgs::msg::EntityType entity_type;

  void setEntityType(uint8_t value) { entity_type.type = value; }

  void appendToJobList(
    const std::function<bool(const double)> & func_on_update,
    const std::function<void()> & func_on_cleanup, traffic_simulator::job::Type type,
    bool exclusive, const traffic_simulator::job::Event event)
  {
    job_list_.append(func_on_update, func_on_cleanup, type, exclusive, event);
  }

  std::optional<double> getTargetSpeed() { return target_speed_; }

public:
  virtual void requestLaneChange(const traffic_simulator::lane_change::Parameter & param) override
  {
    lane_change_param_TEST_ = param;
  }
  auto requestLaneChangeTEST() const -> traffic_simulator::lane_change::Parameter
  {
    return lane_change_param_TEST_;
  }
  auto getOtherstatus() const
    -> const std::unordered_map<std::string, traffic_simulator::CanonicalizedEntityStatus> &
  {
    return other_status_;
  }

  traffic_simulator::lane_change::Parameter lane_change_param_TEST_;
};

#endif