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

#ifndef TRAFFIC_SIMULATOR__ENTITY__DELETED_ENTITY_HPP_
#define TRAFFIC_SIMULATOR__ENTITY__DELETED_ENTITY_HPP_

#include <optional>
#include <traffic_simulator/entity/entity_base.hpp>
#include <traffic_simulator_msgs/msg/misc_object_parameters.hpp>

namespace traffic_simulator
{
namespace entity
{
class DeletedEntity : public EntityBase
{
public:
  // Dummy ID for internal use
  static constexpr uint8_t ENTITY_TYPE_ID = 0xFF;

  explicit DeletedEntity(
    const std::string & name, const CanonicalizedEntityStatus &,
    const std::shared_ptr<hdmap_utils::HdMapUtils> &);

  void onUpdate(double, double) override {}

  auto getCurrentAction() const -> std::string override;

  auto getDefaultDynamicConstraints() const
    -> const traffic_simulator_msgs::msg::DynamicConstraints & override;

  auto getEntityType() const -> const traffic_simulator_msgs::msg::EntityType & override
  {
    static traffic_simulator_msgs::msg::EntityType type;
    type.type = ENTITY_TYPE_ID;  // Dummy ID for internal use
    return type;
  }

  auto getEntityTypename() const -> const std::string & override
  {
    static const std::string result = "DeletedEntity";
    return result;
  }

  ~DeletedEntity() override = default;

  auto getGoalPoses() -> std::vector<CanonicalizedLaneletPose> override { return {}; }

  std::optional<traffic_simulator_msgs::msg::Obstacle> getObstacle() override
  {
    return std::nullopt;
  }

  auto getRouteLanelets(double) -> lanelet::Ids override { return {}; }

  auto fillLaneletPose(CanonicalizedEntityStatus &) -> void override;

  auto getWaypoints() -> const traffic_simulator_msgs::msg::WaypointsArray override
  {
    return traffic_simulator_msgs::msg::WaypointsArray();
  }

  void requestSpeedChange(double, bool) override {}

  void requestSpeedChange(const speed_change::RelativeTargetSpeed &, bool) override {}

  void requestAssignRoute(const std::vector<CanonicalizedLaneletPose> &) override {}

  void requestAssignRoute(const std::vector<geometry_msgs::msg::Pose> &) override {}

  void requestAcquirePosition(const CanonicalizedLaneletPose &) override {}

  void requestAcquirePosition(const geometry_msgs::msg::Pose &) override {}

  void requestSpeedChange(
    const double, const speed_change::Transition, const speed_change::Constraint,
    const bool) override
  {
  }

  auto getBehaviorParameter() const -> traffic_simulator_msgs::msg::BehaviorParameter override;

  void setBehaviorParameter(const traffic_simulator_msgs::msg::BehaviorParameter &) override {}

  void setAccelerationLimit(double) override {}

  void setAccelerationRateLimit(double) override {}

  void setDecelerationLimit(double) override {}

  void setDecelerationRateLimit(double) override {}
};
}  // namespace entity
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__ENTITY__DELETED_ENTITY_HPP_
