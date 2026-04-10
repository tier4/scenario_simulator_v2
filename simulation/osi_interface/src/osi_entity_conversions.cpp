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

#include <cmath>
#include <osi_interface/osi_entity_conversions.hpp>

namespace osi_interface
{
auto toOsiVehicleType(EntitySubtype subtype) -> osi3::MovingObject::VehicleClassification::Type
{
  using VT = osi3::MovingObject::VehicleClassification;
  switch (subtype) {
    case EntitySubtype::CAR:
      return VT::TYPE_MEDIUM_CAR;
    case EntitySubtype::TRUCK:
      return VT::TYPE_HEAVY_TRUCK;
    case EntitySubtype::BUS:
      return VT::TYPE_BUS;
    case EntitySubtype::TRAILER:
      return VT::TYPE_SEMITRAILER;
    case EntitySubtype::MOTORCYCLE:
      return VT::TYPE_MOTORBIKE;
    case EntitySubtype::BICYCLE:
      return VT::TYPE_BICYCLE;
    default:
      return VT::TYPE_UNKNOWN;
  }
}

auto fromOsiVehicleType(osi3::MovingObject::VehicleClassification::Type type) -> EntitySubtype
{
  using VT = osi3::MovingObject::VehicleClassification;
  switch (type) {
    case VT::TYPE_SMALL_CAR:
    case VT::TYPE_MEDIUM_CAR:  // alias of TYPE_CAR
    case VT::TYPE_LUXURY_CAR:
      return EntitySubtype::CAR;
    case VT::TYPE_HEAVY_TRUCK:
    case VT::TYPE_SEMITRACTOR:
      return EntitySubtype::TRUCK;
    case VT::TYPE_BUS:
      return EntitySubtype::BUS;
    case VT::TYPE_SEMITRAILER:
    case VT::TYPE_TRAILER:
      return EntitySubtype::TRAILER;
    case VT::TYPE_MOTORBIKE:  // alias of TYPE_MOTORCYCLE
      return EntitySubtype::MOTORCYCLE;
    case VT::TYPE_BICYCLE:
      return EntitySubtype::BICYCLE;
    default:
      return EntitySubtype::UNKNOWN;
  }
}

auto toOsiTimestamp(double seconds) -> osi3::Timestamp
{
  osi3::Timestamp ts;
  const auto sec = static_cast<int64_t>(std::floor(seconds));
  const auto nanos = static_cast<uint32_t>((seconds - sec) * 1e9);
  ts.set_seconds(sec);
  ts.set_nanos(nanos);
  return ts;
}

auto fromOsiTimestamp(const osi3::Timestamp & ts) -> double
{
  return static_cast<double>(ts.seconds()) + static_cast<double>(ts.nanos()) * 1e-9;
}

namespace
{
void populateBaseMoving(osi3::BaseMoving * base, const EntityData & entity)
{
  // Position (with bounding box center offset)
  auto * pos = base->mutable_position();
  pos->set_x(entity.pose.x + entity.bounding_box.center_x);
  pos->set_y(entity.pose.y + entity.bounding_box.center_y);
  pos->set_z(entity.pose.z + entity.bounding_box.center_z);

  // Orientation (quaternion → euler)
  auto * ori = base->mutable_orientation();
  auto euler = quaternionToEuler({entity.pose.qx, entity.pose.qy, entity.pose.qz, entity.pose.qw});
  ori->set_roll(euler.roll);
  ori->set_pitch(euler.pitch);
  ori->set_yaw(euler.yaw);

  // Dimensions
  auto * dim = base->mutable_dimension();
  dim->set_length(entity.bounding_box.length);
  dim->set_width(entity.bounding_box.width);
  dim->set_height(entity.bounding_box.height);

  // Velocity
  auto * vel = base->mutable_velocity();
  vel->set_x(entity.twist.linear_x);
  vel->set_y(entity.twist.linear_y);
  vel->set_z(entity.twist.linear_z);

  // Orientation rate (angular velocity)
  auto * ori_rate = base->mutable_orientation_rate();
  ori_rate->set_roll(entity.twist.angular_x);
  ori_rate->set_pitch(entity.twist.angular_y);
  ori_rate->set_yaw(entity.twist.angular_z);

  // Acceleration
  auto * acc = base->mutable_acceleration();
  acc->set_x(entity.accel.linear_x);
  acc->set_y(entity.accel.linear_y);
  acc->set_z(entity.accel.linear_z);

  // Orientation acceleration (angular acceleration)
  auto * ori_acc = base->mutable_orientation_acceleration();
  ori_acc->set_roll(entity.accel.angular_x);
  ori_acc->set_pitch(entity.accel.angular_y);
  ori_acc->set_yaw(entity.accel.angular_z);
}

void extractBaseMoving(const osi3::BaseMoving & base, EntityData & entity)
{
  // Position (subtract bounding box center since OSI position = BB center)
  if (base.has_position()) {
    entity.pose.x = base.position().x() - entity.bounding_box.center_x;
    entity.pose.y = base.position().y() - entity.bounding_box.center_y;
    entity.pose.z = base.position().z() - entity.bounding_box.center_z;
  }

  // Orientation (euler → quaternion)
  if (base.has_orientation()) {
    auto q = eulerToQuaternion(
      {base.orientation().roll(), base.orientation().pitch(), base.orientation().yaw()});
    entity.pose.qx = q.x;
    entity.pose.qy = q.y;
    entity.pose.qz = q.z;
    entity.pose.qw = q.w;
  }

  // Dimensions
  if (base.has_dimension()) {
    entity.bounding_box.length = base.dimension().length();
    entity.bounding_box.width = base.dimension().width();
    entity.bounding_box.height = base.dimension().height();
  }

  // Velocity
  if (base.has_velocity()) {
    entity.twist.linear_x = base.velocity().x();
    entity.twist.linear_y = base.velocity().y();
    entity.twist.linear_z = base.velocity().z();
  }

  // Orientation rate
  if (base.has_orientation_rate()) {
    entity.twist.angular_x = base.orientation_rate().roll();
    entity.twist.angular_y = base.orientation_rate().pitch();
    entity.twist.angular_z = base.orientation_rate().yaw();
  }

  // Acceleration
  if (base.has_acceleration()) {
    entity.accel.linear_x = base.acceleration().x();
    entity.accel.linear_y = base.acceleration().y();
    entity.accel.linear_z = base.acceleration().z();
  }

  // Orientation acceleration
  if (base.has_orientation_acceleration()) {
    entity.accel.angular_x = base.orientation_acceleration().roll();
    entity.accel.angular_y = base.orientation_acceleration().pitch();
    entity.accel.angular_z = base.orientation_acceleration().yaw();
  }
}
}  // namespace

auto toOsiMovingObject(const EntityData & entity, EntityIdRegistry & registry) -> osi3::MovingObject
{
  osi3::MovingObject obj;

  // ID
  *obj.mutable_id() = registry.assign(entity.name);

  // Type
  if (entity.type == EntityType::PEDESTRIAN) {
    obj.set_type(osi3::MovingObject::TYPE_PEDESTRIAN);
  } else {
    obj.set_type(osi3::MovingObject::TYPE_VEHICLE);
    obj.mutable_vehicle_classification()->set_type(toOsiVehicleType(entity.subtype));
  }

  // Base (position, orientation, velocity, acceleration, dimensions)
  populateBaseMoving(obj.mutable_base(), entity);

  // Source reference (store entity name for reverse lookup)
  auto * ref = obj.add_source_reference();
  ref->set_type("net.asam.openscenario");
  ref->add_identifier(entity.name);
  if (!entity.asset_key.empty()) {
    obj.set_model_reference(entity.asset_key);
  }

  return obj;
}

auto fromOsiMovingObject(const osi3::MovingObject & obj, const EntityIdRegistry & registry)
  -> EntityData
{
  EntityData entity;

  // Name from registry
  if (obj.has_id()) {
    auto name = registry.reverseLookup(obj.id().value());
    if (name.has_value()) {
      entity.name = *name;
    }
  }

  // Type
  if (obj.type() == osi3::MovingObject::TYPE_PEDESTRIAN) {
    entity.type = EntityType::PEDESTRIAN;
    entity.subtype = EntitySubtype::PEDESTRIAN;
  } else {
    entity.type = EntityType::VEHICLE;
    if (obj.has_vehicle_classification()) {
      entity.subtype = fromOsiVehicleType(obj.vehicle_classification().type());
    }
  }

  // Base
  if (obj.has_base()) {
    extractBaseMoving(obj.base(), entity);
  }

  // Model reference
  if (obj.has_model_reference()) {
    entity.asset_key = obj.model_reference();
  }

  return entity;
}

auto toOsiStationaryObject(const EntityData & entity, EntityIdRegistry & registry)
  -> osi3::StationaryObject
{
  osi3::StationaryObject obj;

  *obj.mutable_id() = registry.assign(entity.name);

  auto * base = obj.mutable_base();
  auto * pos = base->mutable_position();
  pos->set_x(entity.pose.x + entity.bounding_box.center_x);
  pos->set_y(entity.pose.y + entity.bounding_box.center_y);
  pos->set_z(entity.pose.z + entity.bounding_box.center_z);

  auto * ori = base->mutable_orientation();
  auto euler = quaternionToEuler({entity.pose.qx, entity.pose.qy, entity.pose.qz, entity.pose.qw});
  ori->set_roll(euler.roll);
  ori->set_pitch(euler.pitch);
  ori->set_yaw(euler.yaw);

  auto * dim = base->mutable_dimension();
  dim->set_length(entity.bounding_box.length);
  dim->set_width(entity.bounding_box.width);
  dim->set_height(entity.bounding_box.height);

  auto * ref = obj.add_source_reference();
  ref->set_type("net.asam.openscenario");
  ref->add_identifier(entity.name);

  if (!entity.asset_key.empty()) {
    obj.set_model_reference(entity.asset_key);
  }

  return obj;
}

auto fromOsiStationaryObject(const osi3::StationaryObject & obj, const EntityIdRegistry & registry)
  -> EntityData
{
  EntityData entity;
  entity.type = EntityType::MISC_OBJECT;

  if (obj.has_id()) {
    auto name = registry.reverseLookup(obj.id().value());
    if (name.has_value()) {
      entity.name = *name;
    }
  }

  if (obj.has_base()) {
    const auto & base = obj.base();
    if (base.has_position()) {
      entity.pose.x = base.position().x();
      entity.pose.y = base.position().y();
      entity.pose.z = base.position().z();
    }
    if (base.has_orientation()) {
      auto q = eulerToQuaternion(
        {base.orientation().roll(), base.orientation().pitch(), base.orientation().yaw()});
      entity.pose.qx = q.x;
      entity.pose.qy = q.y;
      entity.pose.qz = q.z;
      entity.pose.qw = q.w;
    }
    if (base.has_dimension()) {
      entity.bounding_box.length = base.dimension().length();
      entity.bounding_box.width = base.dimension().width();
      entity.bounding_box.height = base.dimension().height();
    }
  }

  if (obj.has_model_reference()) {
    entity.asset_key = obj.model_reference();
  }

  return entity;
}

}  // namespace osi_interface
