// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef SIMULATION_CONTROLLER__ENTITY__ENTITY_BASE_HPP_
#define SIMULATION_CONTROLLER__ENTITY__ENTITY_BASE_HPP_

#include <simulation_controller/entity/entity_status.hpp>
#include <simulation_controller/hdmap_utils/hdmap_utils.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/optional.hpp>

// headers in STL
#include <string>
#include <memory>
#include <unordered_map>

namespace simulation_controller
{
namespace entity
{
enum class Direction
{
  STRAIGHT = 0,
  LEFT = 1,
  RIGHT = 2
};

enum class EntityType
{
  EGO = 0,
  VEHICLE = 1,
  PEDESTRIAN = 2
};

class EntityBase
{
public:
  EntityBase(std::string type, std::string name, const EntityStatus & initial_state);
  EntityBase(std::string type, std::string name);
  virtual ~EntityBase() = default;
  const std::string type;
  const std::string name;
  const CoordinateFrameTypes & getStatusCoordinateFrameType() const;
  const EntityStatus getStatus(CoordinateFrameTypes coordinate = CoordinateFrameTypes::WORLD) const;
  bool setStatus(const EntityStatus & status);
  bool setVisibility(bool visibility);
  bool getVisibility();
  void setHdMapUtils(std::shared_ptr<hdmap_utils::HdMapUtils> ptr)
  {
    hdmap_utils_ptr_ = ptr;
  }
  virtual void onUpdate(double current_time, double step_time) = 0;
  bool statusSetted() const
  {
    if (status_) {
      return true;
    }
    return false;
  }
  void setVerbose(bool verbose)
  {
    verbose_ = verbose;
  }
  void setEntityTypeList(const std::unordered_map<std::string, EntityType> & entity_type_list)
  {
    entity_type_list_ = entity_type_list;
  }
  void setOtherStatus(const std::unordered_map<std::string, EntityStatus> & status);
  void updateStandStillDuration(double step_time);
  boost::optional<double> getStandStillDuration() const;

protected:
  bool visibility_;
  boost::optional<EntityStatus> status_;
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr_;
  bool verbose_;
  std::unordered_map<std::string, EntityStatus> other_status_;
  std::unordered_map<std::string, EntityType> entity_type_list_;
  boost::optional<double> stand_still_duration_;
};
}  // namespace entity
}  // namespace simulation_controller

#endif  // SIMULATION_CONTROLLER__ENTITY__ENTITY_BASE_HPP_
