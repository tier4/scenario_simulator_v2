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

#include <gtest/gtest.h>

#include <cmath>
#include <osi_interface/osi_entity_conversions.hpp>

using namespace osi_interface;

constexpr double kTol = 1e-9;

EntityData makeVehicle(const std::string & name)
{
  EntityData e;
  e.name = name;
  e.type = EntityType::VEHICLE;
  e.subtype = EntitySubtype::CAR;
  e.pose = {1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0};
  e.twist = {10.0, 0.5, 0.0, 0.0, 0.0, 0.1};
  e.accel = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  e.bounding_box = {0.0, 0.0, 0.0, 4.5, 1.8, 1.5};
  return e;
}

TEST(OsiEntityConversions, VehicleRoundTrip)
{
  EntityIdRegistry registry;
  auto entity = makeVehicle("npc_car");
  auto obj = toOsiMovingObject(entity, registry);
  auto result = fromOsiMovingObject(obj, registry);

  EXPECT_EQ(result.name, "npc_car");
  EXPECT_EQ(result.type, EntityType::VEHICLE);
  EXPECT_EQ(result.subtype, EntitySubtype::CAR);
  EXPECT_NEAR(result.pose.x, entity.pose.x, kTol);
  EXPECT_NEAR(result.pose.y, entity.pose.y, kTol);
  EXPECT_NEAR(result.pose.z, entity.pose.z, kTol);
  EXPECT_NEAR(result.twist.linear_x, entity.twist.linear_x, kTol);
  EXPECT_NEAR(result.accel.linear_x, entity.accel.linear_x, kTol);
  EXPECT_NEAR(result.bounding_box.length, 4.5, kTol);
  EXPECT_NEAR(result.bounding_box.width, 1.8, kTol);
}

TEST(OsiEntityConversions, PedestrianType)
{
  EntityIdRegistry registry;
  EntityData entity;
  entity.name = "ped_1";
  entity.type = EntityType::PEDESTRIAN;
  entity.subtype = EntitySubtype::PEDESTRIAN;
  entity.pose = {5.0, 6.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  entity.bounding_box = {0.0, 0.0, 0.0, 0.5, 0.5, 1.7};

  auto obj = toOsiMovingObject(entity, registry);
  EXPECT_EQ(obj.type(), osi3::MovingObject::TYPE_PEDESTRIAN);

  auto result = fromOsiMovingObject(obj, registry);
  EXPECT_EQ(result.type, EntityType::PEDESTRIAN);
}

TEST(OsiEntityConversions, MiscObjectAsStationary)
{
  EntityIdRegistry registry;
  EntityData entity;
  entity.name = "barrier_1";
  entity.type = EntityType::MISC_OBJECT;
  entity.pose = {10.0, 20.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  entity.bounding_box = {0.0, 0.0, 0.0, 2.0, 0.5, 1.0};

  auto obj = toOsiStationaryObject(entity, registry);
  EXPECT_TRUE(obj.has_id());
  EXPECT_TRUE(obj.has_base());
  EXPECT_NEAR(obj.base().dimension().length(), 2.0, kTol);

  auto result = fromOsiStationaryObject(obj, registry);
  EXPECT_EQ(result.name, "barrier_1");
  EXPECT_EQ(result.type, EntityType::MISC_OBJECT);
  EXPECT_NEAR(result.pose.x, 10.0, kTol);
}

TEST(OsiEntityConversions, BoundingBoxCenterOffset)
{
  EntityIdRegistry registry;
  EntityData entity;
  entity.name = "offset_car";
  entity.type = EntityType::VEHICLE;
  entity.subtype = EntitySubtype::CAR;
  entity.pose = {100.0, 200.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  entity.bounding_box = {0.5, 0.0, 0.3, 4.5, 1.8, 1.5};  // center offset

  auto obj = toOsiMovingObject(entity, registry);
  // OSI position should be pose + center offset
  EXPECT_NEAR(obj.base().position().x(), 100.5, kTol);
  EXPECT_NEAR(obj.base().position().z(), 0.3, kTol);

  auto result = fromOsiMovingObject(obj, registry);
  // After round-trip, BB center offset is lost (becomes 0),
  // but pose should be at the OSI position
  EXPECT_NEAR(result.pose.x, obj.base().position().x(), kTol);
}

TEST(OsiEntityConversions, OrientationConversion)
{
  EntityIdRegistry registry;
  EntityData entity;
  entity.name = "rotated_car";
  entity.type = EntityType::VEHICLE;
  entity.subtype = EntitySubtype::CAR;
  // Quaternion for yaw=pi/4
  double yaw = M_PI / 4.0;
  entity.pose.qz = std::sin(yaw / 2.0);
  entity.pose.qw = std::cos(yaw / 2.0);
  entity.bounding_box = {0.0, 0.0, 0.0, 4.5, 1.8, 1.5};

  auto obj = toOsiMovingObject(entity, registry);
  EXPECT_NEAR(obj.base().orientation().yaw(), M_PI / 4.0, kTol);

  auto result = fromOsiMovingObject(obj, registry);
  EXPECT_NEAR(result.pose.qz, entity.pose.qz, kTol);
  EXPECT_NEAR(result.pose.qw, entity.pose.qw, kTol);
}

TEST(OsiEntityConversions, AllSubtypeMapping)
{
  using VT = osi3::MovingObject::VehicleClassification;
  EXPECT_EQ(toOsiVehicleType(EntitySubtype::CAR), VT::TYPE_MEDIUM_CAR);
  EXPECT_EQ(toOsiVehicleType(EntitySubtype::TRUCK), VT::TYPE_HEAVY_TRUCK);
  EXPECT_EQ(toOsiVehicleType(EntitySubtype::BUS), VT::TYPE_BUS);
  EXPECT_EQ(toOsiVehicleType(EntitySubtype::TRAILER), VT::TYPE_SEMITRAILER);
  EXPECT_EQ(toOsiVehicleType(EntitySubtype::MOTORCYCLE), VT::TYPE_MOTORBIKE);
  EXPECT_EQ(toOsiVehicleType(EntitySubtype::BICYCLE), VT::TYPE_BICYCLE);
  EXPECT_EQ(toOsiVehicleType(EntitySubtype::UNKNOWN), VT::TYPE_UNKNOWN);

  EXPECT_EQ(fromOsiVehicleType(VT::TYPE_MEDIUM_CAR), EntitySubtype::CAR);
  EXPECT_EQ(fromOsiVehicleType(VT::TYPE_SMALL_CAR), EntitySubtype::CAR);
  EXPECT_EQ(fromOsiVehicleType(VT::TYPE_HEAVY_TRUCK), EntitySubtype::TRUCK);
  EXPECT_EQ(fromOsiVehicleType(VT::TYPE_BUS), EntitySubtype::BUS);
  EXPECT_EQ(fromOsiVehicleType(VT::TYPE_SEMITRAILER), EntitySubtype::TRAILER);
  EXPECT_EQ(fromOsiVehicleType(VT::TYPE_MOTORBIKE), EntitySubtype::MOTORCYCLE);
  EXPECT_EQ(fromOsiVehicleType(VT::TYPE_BICYCLE), EntitySubtype::BICYCLE);
}

TEST(OsiEntityConversions, TimestampConversion)
{
  auto ts = toOsiTimestamp(1.5);
  EXPECT_EQ(ts.seconds(), 1);
  EXPECT_EQ(ts.nanos(), 500000000u);

  double back = fromOsiTimestamp(ts);
  EXPECT_NEAR(back, 1.5, 1e-9);
}

TEST(OsiEntityConversions, TimestampZero)
{
  auto ts = toOsiTimestamp(0.0);
  EXPECT_EQ(ts.seconds(), 0);
  EXPECT_EQ(ts.nanos(), 0u);
  EXPECT_NEAR(fromOsiTimestamp(ts), 0.0, 1e-15);
}

TEST(OsiEntityConversions, SourceReferenceContainsName)
{
  EntityIdRegistry registry;
  auto entity = makeVehicle("test_source_ref");
  entity.asset_key = "my_model.fbx";
  auto obj = toOsiMovingObject(entity, registry);

  ASSERT_GE(obj.source_reference_size(), 1);
  EXPECT_EQ(obj.source_reference(0).type(), "net.asam.openscenario");
  ASSERT_GE(obj.source_reference(0).identifier_size(), 1);
  EXPECT_EQ(obj.source_reference(0).identifier(0), "test_source_ref");
  EXPECT_EQ(obj.model_reference(), "my_model.fbx");
}
