// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#ifndef TRAFFIC_SIMULATOR__ENTITY__MISC_OBJECT_ENTITY_HPP_
#define TRAFFIC_SIMULATOR__ENTITY__MISC_OBJECT_ENTITY_HPP_

#include <traffic_simulator/entity/entity_base.hpp>
#include <traffic_simulator_msgs/msg/misc_object_parameters.hpp>

namespace traffic_simulator
{
namespace entity
{
class MiscObjectEntity : public EntityBase
{
public:
  MiscObjectEntity(
    const std::string & name, const traffic_simulator_msgs::msg::MiscObjectParameters & params);
  void onUpdate(double, double) override;
  auto getBoundingBox() const -> const traffic_simulator_msgs::msg::BoundingBox override;
  auto getCurrentAction() const -> const std::string override;
  auto getEntityTypename() const -> const std::string & override
  {
    static const std::string result = "VehicleEntity";
    return result;
  }

  std::vector<traffic_simulator_msgs::msg::LaneletPose> getGoalPoses() override
  {
    return {};
  }  //return {}?

  boost::optional<traffic_simulator_msgs::msg::Obstacle> getObstacle() override
  {
    return boost::none;
  }

  auto getRouteLanelets(const double) -> std::vector<std::int64_t> override
  {
    THROW_SEMANTIC_ERROR("getRouteLanelets function cannot not use in MiscObjectEntity");
  }

  auto getWaypoints() -> const traffic_simulator_msgs::msg::WaypointsArray override
  {
    return traffic_simulator_msgs::msg::WaypointsArray();
  }

  void setTargetSpeed(double, bool) override
  {
    THROW_SEMANTIC_ERROR("setTargetSpeed function cannot not use in MiscObjectEntity");
  }

  void requestAssignRoute(const std::vector<traffic_simulator_msgs::msg::LaneletPose> &) override
  {
    THROW_SEMANTIC_ERROR("requestAssignRoute function cannot not use in MiscObjectEntity");
  }

  void requestAssignRoute(const std::vector<geometry_msgs::msg::Pose> &) override
  {
    THROW_SEMANTIC_ERROR("requestAssignRoute function cannot not use in MiscObjectEntity");
  }

  void requestAcquirePosition(const traffic_simulator_msgs::msg::LaneletPose &) override
  {
    THROW_SEMANTIC_ERROR("requestAcquirePosition function cannot not use in MiscObjectEntity");
  }

  void requestAcquirePosition(const geometry_msgs::msg::Pose &) override
  {
    THROW_SEMANTIC_ERROR("requestAcquirePosition function cannot not use in MiscObjectEntity");
  }

  void requestSpeedChange(
    const double, const SpeedChangeTransition, const SpeedChangeConstraint, const bool) override
  {
    THROW_SEMANTIC_ERROR("requestSpeedChange function cannot not use in MiscObjectEntity");
  }

  auto getDriverModel() const -> traffic_simulator_msgs::msg::DriverModel override;

  void setDriverModel(const traffic_simulator_msgs::msg::DriverModel &) override;

private:
  const traffic_simulator_msgs::msg::MiscObjectParameters params_;
};
}  // namespace entity
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__ENTITY__MISC_OBJECT_ENTITY_HPP_
