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

#include <string>
#include <traffic_simulator/math/distance.hpp>
#include <traffic_simulator/math/polygon.hpp>
#include <traffic_simulator/math/transform.hpp>
#include <traffic_simulator/metrics/pull_over_metric.hpp>

namespace metrics
{
PullOverMetric::PullOverMetric(
  const std::string & target_entity, std::int64_t target_lanelet_id,
  double threshold_standstill_duration, double threshold_yaw, double threshold_lateral_distance)
: MetricBase("PullOver"),
  target_entity(target_entity),
  target_lanelet_id(target_lanelet_id),
  threshold_standstill_duration(threshold_standstill_duration),
  threshold_yaw(threshold_yaw),
  threshold_lateral_distance(threshold_lateral_distance)
{
  if (threshold_yaw < 0) {
    THROW_SEMANTIC_ERROR(
      "threshold_yaw value in PullOverMetric should be over 0, value : ", threshold_yaw);
  }
  if (threshold_lateral_distance < 0) {
    THROW_SEMANTIC_ERROR(
      "threshold_lateral_yaw value in PullOverMetric should be over 0, value : ", threshold_yaw);
  }
}

void PullOverMetric::update()
{
  if (entity_manager_ptr_->getStandStillDuration(target_entity) >= threshold_standstill_duration) {
    const auto lanelet_pose = entity_manager_ptr_->getLaneletPose(target_entity);
    if (lanelet_pose && std::abs(lanelet_pose->rpy.z) <= threshold_yaw) {
      const auto polygon = traffic_simulator::math::transformPoints(
        entity_manager_ptr_->getMapPose(target_entity),
        entity_manager_ptr_->get2DPolygon(target_entity));
      if (polygon.empty()) {
        THROW_SEMANTIC_ERROR(
          "Failed to calculate 2d polygon of entity: ", target_entity, " . Please check ",
          target_entity, " exists.");
      }
      const auto left_bound = entity_manager_ptr_->getHdmapUtils()->getLeftBound(target_lanelet_id);
      const auto right_bound =
        entity_manager_ptr_->getHdmapUtils()->getRightBound(target_lanelet_id);
      if (left_bound.empty()) {
        THROW_SEMANTIC_ERROR(
          "Failed to calculate left bounds of lanelet_id : ", target_lanelet_id,
          " please check lanelet map.");
      }
      if (right_bound.empty()) {
        THROW_SEMANTIC_ERROR(
          "Failed to calculate right bounds of lanelet_id : ", target_lanelet_id,
          " please check lanelet map.");
      }
      const auto left_distance = traffic_simulator::math::getDistance2D(left_bound, polygon);
      const auto right_distance = traffic_simulator::math::getDistance2D(right_bound, polygon);
      if (
        left_distance <= threshold_lateral_distance ||
        right_distance <= threshold_lateral_distance) {
        success();
      } else {
        if (left_distance > threshold_lateral_distance) {
          failure(SPECIFICATION_VIOLATION(
            "pulling over entity : ", target_entity,
            " was failed because of lateral distance to the left bound is ", left_distance,
            "is over threshold :", threshold_lateral_distance));
          return;
        }
        if (right_distance > threshold_lateral_distance) {
          failure(SPECIFICATION_VIOLATION(
            "pulling over entity : ", target_entity,
            " was failed because of lateral distance to the right bound is ", right_distance,
            "is over threshold :", threshold_lateral_distance));
          return;
        }
      }
    } else {
      failure(SPECIFICATION_VIOLATION(
        "pulling over entity : ", target_entity,
        " was failed because of this entity stops diagonally."));
      return;
    }
  }
}

nlohmann::json PullOverMetric::toJson()
{
  nlohmann::json json = MetricBase::toBaseJson();
  const auto standstill_duration = entity_manager_ptr_->getStandStillDuration(target_entity);
  if (standstill_duration) {
    json["standstill_duration"] = standstill_duration.get();
  }
  if (getLifecycle() != MetricLifecycle::ACTIVE) {
    const auto lanelet_pose = entity_manager_ptr_->getLaneletPose(target_entity);
    if (lanelet_pose) {
      json["yaw"] = lanelet_pose->rpy.z;
    }
  }
  return json;
}

bool PullOverMetric::activateTrigger()
{
  return entity_manager_ptr_->isInLanelet(target_entity, target_lanelet_id, 1.0);
}
}  // namespace metrics
