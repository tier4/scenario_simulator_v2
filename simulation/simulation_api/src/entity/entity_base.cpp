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

#include <simulation_api/entity/entity_base.hpp>
#include <simulation_api/entity/exception.hpp>

#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <limits>
#include <string>

namespace simulation_api
{
namespace entity
{
EntityBase::EntityBase(std::string type, std::string name, const EntityStatus & initial_state)
: type(type), name(name)
{
  status_ = initial_state;
  visibility_ = true;
  verbose_ = true;
}

EntityBase::EntityBase(std::string type, std::string name)
: type(type), name(name)
{
  status_ = boost::none;
  visibility_ = true;
  verbose_ = true;
}

boost::optional<double> EntityBase::getStandStillDuration() const
{
  return stand_still_duration_;
}

void EntityBase::updateStandStillDuration(double step_time)
{
  if (!status_) {
    stand_still_duration_ = boost::none;
  } else {
    if (!stand_still_duration_) {
      stand_still_duration_ = 0;
    }
    if (std::fabs(status_->twist.linear.x) <= std::numeric_limits<double>::epsilon()) {
      stand_still_duration_ = step_time + stand_still_duration_.get();
    } else {
      stand_still_duration_ = 0;
    }
  }
}

void EntityBase::setOtherStatus(const std::unordered_map<std::string, EntityStatus> & status)
{
  std::unordered_map<std::string, EntityStatus> other_status;
  for (const auto & each : status) {
    if (each.first != name) {
      other_status.insert(each);
    }
  }
  other_status_ = other_status;
}

const EntityStatus EntityBase::getStatus(CoordinateFrameTypes coordinate) const
{
  if (!status_) {
    throw SimulationRuntimeError("status is not set");
  }
  if (coordinate == this->status_->coordinate) {
    return this->status_.get();
  }
  if (coordinate == CoordinateFrameTypes::LANE) {
    if (this->status_->coordinate == CoordinateFrameTypes::WORLD) {
      auto lane_pose = hdmap_utils_ptr_->toLanePose(this->status_->pose);
      if (lane_pose) {
        throw SimulationRuntimeError("Failed to calculate pose from world to lane");
      }
      return EntityStatus(this->status_->time, lane_pose->lanelet_id, lane_pose->s,
               lane_pose->offset, lane_pose->rpy, this->status_->twist,
               this->status_->accel);
    }
  }
  if (coordinate == CoordinateFrameTypes::WORLD) {
    if (this->status_->coordinate == CoordinateFrameTypes::LANE) {
      auto map_pose = hdmap_utils_ptr_->toMapPose(this->status_->lanelet_id, this->status_->s,
          this->status_->offset, this->status_->rpy);
      if (map_pose) {
        return EntityStatus(this->status_->time, map_pose->pose, this->status_->twist,
                 this->status_->accel);
      } else {
        throw SimulationRuntimeError("Failed to calculate pose from lane to world");
      }
    }
  }
  throw SimulationRuntimeError("CoordinateFrameTypes of the entity status does not match");
}

bool EntityBase::setStatus(const EntityStatus & status)
{
  if (status.coordinate == CoordinateFrameTypes::LANE) {
    if (hdmap_utils_ptr_->isInLanelet(status.lanelet_id, status.s)) {
      this->status_ = status;
      return true;
    } else {
      auto ids = hdmap_utils_ptr_->getNextLaneletIds(status.lanelet_id, "straight");
      if (ids.size() == 0) {
        auto following_ids = hdmap_utils_ptr_->getNextLaneletIds(status.lanelet_id);
        if (following_ids.size() == 0) {
          if (status.coordinate == CoordinateFrameTypes::LANE) {
            this->status_ = boost::none;
            return true;
          } else {
            throw SimulationRuntimeError("failed to calculate map pose at the end of the lanelet");
          }
        }
        if (status.coordinate == CoordinateFrameTypes::LANE) {
          double l = status.s - hdmap_utils_ptr_->getLaneletLength(status.lanelet_id);
          if (l < 0.0) {
            l = 0.0;
          }
          this->status_ = EntityStatus(this->status_->time, following_ids[0], l,
              this->status_->offset, this->status_->rpy,
              this->status_->twist, this->status_->accel);
          return true;
        }
        throw SimulationRuntimeError("failed to calculate map pose at the end of the lanelet");
      }
      double l = status.s - hdmap_utils_ptr_->getLaneletLength(status.lanelet_id);
      if (l < 0.0) {
        l = 0.0;
      }
      this->status_ = EntityStatus(this->status_->time, ids[0], l, this->status_->offset,
          this->status_->rpy, this->status_->twist, this->status_->accel);
      return true;
    }
  }
  this->status_ = status;
  return true;
}

const CoordinateFrameTypes & EntityBase::getStatusCoordinateFrameType() const
{
  return this->status_->coordinate;
}

bool EntityBase::setVisibility(bool visibility)
{
  visibility_ = visibility;
  return visibility_;
}

bool EntityBase::getVisibility()
{
  return visibility_;
}
}  // namespace entity
}  // namespace simulation_api
