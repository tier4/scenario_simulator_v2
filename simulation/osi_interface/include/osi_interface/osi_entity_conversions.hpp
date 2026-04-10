// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#ifndef OSI_INTERFACE__OSI_ENTITY_CONVERSIONS_HPP_
#define OSI_INTERFACE__OSI_ENTITY_CONVERSIONS_HPP_

#include <osi3/osi_common.pb.h>
#include <osi3/osi_object.pb.h>

#include <osi_interface/entity_id_registry.hpp>
#include <osi_interface/orientation_conversion.hpp>
#include <string>

namespace osi_interface
{
// Lightweight struct mirroring the data needed for conversion,
// decoupled from simulation_api_schema protobuf types.
struct EntityPose
{
  double x{0}, y{0}, z{0};
  double qx{0}, qy{0}, qz{0}, qw{1};
};

struct EntityTwist
{
  double linear_x{0}, linear_y{0}, linear_z{0};
  double angular_x{0}, angular_y{0}, angular_z{0};
};

struct EntityAccel
{
  double linear_x{0}, linear_y{0}, linear_z{0};
  double angular_x{0}, angular_y{0}, angular_z{0};
};

struct EntityBoundingBox
{
  double center_x{0}, center_y{0}, center_z{0};
  double length{0}, width{0}, height{0};
};

enum class EntityType { EGO = 0, VEHICLE = 1, PEDESTRIAN = 2, MISC_OBJECT = 3 };

enum class EntitySubtype {
  UNKNOWN = 0,
  CAR = 1,
  TRUCK = 2,
  BUS = 3,
  TRAILER = 4,
  MOTORCYCLE = 5,
  BICYCLE = 6,
  PEDESTRIAN = 7
};

struct EntityData
{
  std::string name;
  EntityType type{EntityType::VEHICLE};
  EntitySubtype subtype{EntitySubtype::UNKNOWN};
  EntityPose pose;
  EntityTwist twist;
  EntityAccel accel;
  EntityBoundingBox bounding_box;
  std::string asset_key;
};

// Convert EntityData → osi3::MovingObject (for VEHICLE, PEDESTRIAN, EGO)
auto toOsiMovingObject(const EntityData & entity, EntityIdRegistry & registry)
  -> osi3::MovingObject;

// Convert osi3::MovingObject → EntityData
auto fromOsiMovingObject(const osi3::MovingObject & obj, const EntityIdRegistry & registry)
  -> EntityData;

// Convert EntityData → osi3::StationaryObject (for MISC_OBJECT)
auto toOsiStationaryObject(const EntityData & entity, EntityIdRegistry & registry)
  -> osi3::StationaryObject;

// Convert osi3::StationaryObject → EntityData
auto fromOsiStationaryObject(const osi3::StationaryObject & obj, const EntityIdRegistry & registry)
  -> EntityData;

// Map EntitySubtype → osi3 VehicleClassification::Type
auto toOsiVehicleType(EntitySubtype subtype) -> osi3::MovingObject::VehicleClassification::Type;

// Map osi3 VehicleClassification::Type → EntitySubtype
auto fromOsiVehicleType(osi3::MovingObject::VehicleClassification::Type type) -> EntitySubtype;

// Convert simulation time (double seconds) → osi3::Timestamp
auto toOsiTimestamp(double seconds) -> osi3::Timestamp;

// Convert osi3::Timestamp → double seconds
auto fromOsiTimestamp(const osi3::Timestamp & ts) -> double;

}  // namespace osi_interface

#endif  // OSI_INTERFACE__OSI_ENTITY_CONVERSIONS_HPP_
