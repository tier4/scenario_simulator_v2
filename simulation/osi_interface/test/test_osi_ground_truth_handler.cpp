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

#include <osi_interface/osi_ground_truth_handler.hpp>

using namespace osi_interface;

constexpr double kTol = 1e-9;

// Helper: build a GroundTruth with one ego and one NPC
osi3::GroundTruth makeSimpleGroundTruth(EntityIdRegistry & registry)
{
  std::vector<EntityData> moving;

  EntityData ego;
  ego.name = "ego";
  ego.type = EntityType::EGO;
  ego.subtype = EntitySubtype::CAR;
  ego.pose = {10.0, 20.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  ego.twist = {5.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  ego.bounding_box = {0.0, 0.0, 0.0, 4.5, 1.8, 1.5};
  moving.push_back(ego);

  EntityData npc;
  npc.name = "npc_1";
  npc.type = EntityType::VEHICLE;
  npc.subtype = EntitySubtype::TRUCK;
  npc.pose = {30.0, 40.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  npc.bounding_box = {0.0, 0.0, 0.0, 8.0, 2.5, 3.0};
  moving.push_back(npc);

  return GroundTruthBuilder(registry)
    .setTimestamp(1.0)
    .setHostVehicle("ego")
    .setMapReference("/path/to/map.osm")
    .setMovingEntities(moving)
    .build();
}

TEST(OsiGroundTruthHandler, ParseBasicGroundTruth)
{
  // Build a GroundTruth using a separate registry (simulating traffic_simulator side)
  EntityIdRegistry build_registry;
  auto gt = makeSimpleGroundTruth(build_registry);

  // Parse it using the handler (simulating simple_sensor_simulator side)
  // The handler's registry needs to know the ID→name mapping.
  // In real usage, the IDs are embedded in the GroundTruth's ExternalReference.
  // For this test, we pre-populate the handler's registry.
  OsiGroundTruthHandler handler;
  handler.getRegistry().assign("ego");
  handler.getRegistry().assign("npc_1");

  auto frame = handler.parseGroundTruth(gt);

  EXPECT_NEAR(frame.simulation_time, 1.0, kTol);
  EXPECT_EQ(frame.ego_name, "ego");
  EXPECT_EQ(frame.map_reference, "/path/to/map.osm");
  ASSERT_EQ(frame.moving_entities.size(), 2u);
}

TEST(OsiGroundTruthHandler, DetectSpawnOnFirstFrame)
{
  EntityIdRegistry build_registry;
  auto gt = makeSimpleGroundTruth(build_registry);

  OsiGroundTruthHandler handler;
  // Don't pre-populate registry — IDs will be assigned during parse via fromOsiMovingObject
  // which uses ExternalReference to extract names.
  // But since our test GroundTruth has ExternalReference set by toOsiMovingObject,
  // fromOsiMovingObject will try reverseLookup and may fail.
  // Pre-populate with same IDs as build_registry for consistency.
  handler.getRegistry().assign("ego");
  handler.getRegistry().assign("npc_1");

  auto frame = handler.parseGroundTruth(gt);

  // First frame: all entities are "spawned"
  EXPECT_EQ(frame.spawned_moving.size(), 2u);
  EXPECT_EQ(frame.despawned_names.size(), 0u);
}

TEST(OsiGroundTruthHandler, DetectDespawn)
{
  EntityIdRegistry build_registry;

  // Frame 1: ego + npc_1
  auto gt1 = makeSimpleGroundTruth(build_registry);

  // Frame 2: ego only (npc_1 removed)
  std::vector<EntityData> moving2;
  EntityData ego;
  ego.name = "ego";
  ego.type = EntityType::EGO;
  ego.subtype = EntitySubtype::CAR;
  ego.pose = {11.0, 20.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  ego.bounding_box = {0.0, 0.0, 0.0, 4.5, 1.8, 1.5};
  moving2.push_back(ego);

  auto gt2 = GroundTruthBuilder(build_registry)
               .setTimestamp(2.0)
               .setHostVehicle("ego")
               .setMovingEntities(moving2)
               .build();

  OsiGroundTruthHandler handler;
  handler.getRegistry().assign("ego");
  handler.getRegistry().assign("npc_1");

  // Process frame 1
  handler.parseGroundTruth(gt1);

  // Process frame 2
  auto frame2 = handler.parseGroundTruth(gt2);

  EXPECT_EQ(frame2.moving_entities.size(), 1u);
  EXPECT_EQ(frame2.spawned_moving.size(), 0u);
  ASSERT_EQ(frame2.despawned_names.size(), 1u);
  EXPECT_EQ(frame2.despawned_names[0], "npc_1");
}

TEST(OsiGroundTruthHandler, DetectNewSpawnInLaterFrame)
{
  EntityIdRegistry build_registry;

  // Frame 1: ego only
  std::vector<EntityData> moving1;
  EntityData ego;
  ego.name = "ego";
  ego.type = EntityType::EGO;
  ego.subtype = EntitySubtype::CAR;
  ego.pose = {10.0, 20.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  ego.bounding_box = {0.0, 0.0, 0.0, 4.5, 1.8, 1.5};
  moving1.push_back(ego);

  auto gt1 = GroundTruthBuilder(build_registry)
               .setTimestamp(1.0)
               .setHostVehicle("ego")
               .setMovingEntities(moving1)
               .build();

  // Frame 2: ego + npc_1 (new spawn)
  auto gt2 = makeSimpleGroundTruth(build_registry);

  OsiGroundTruthHandler handler;
  handler.getRegistry().assign("ego");
  handler.getRegistry().assign("npc_1");

  handler.parseGroundTruth(gt1);
  auto frame2 = handler.parseGroundTruth(gt2);

  ASSERT_EQ(frame2.spawned_moving.size(), 1u);
  EXPECT_EQ(frame2.spawned_moving[0].name, "npc_1");
  EXPECT_EQ(frame2.despawned_names.size(), 0u);
}

TEST(OsiGroundTruthHandler, BuildTrafficUpdate)
{
  OsiGroundTruthHandler handler;

  EntityData ego;
  ego.name = "ego";
  ego.type = EntityType::EGO;
  ego.subtype = EntitySubtype::CAR;
  ego.pose = {15.0, 25.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  ego.twist = {8.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  ego.bounding_box = {0.0, 0.0, 0.0, 4.5, 1.8, 1.5};

  auto tu = handler.buildTrafficUpdate(3.0, ego);

  EXPECT_NEAR(fromOsiTimestamp(tu.timestamp()), 3.0, kTol);
  ASSERT_EQ(tu.update_size(), 1);
  EXPECT_NEAR(tu.update(0).base().position().x(), 15.0, kTol);
  EXPECT_NEAR(tu.update(0).base().velocity().x(), 8.0, kTol);
}

TEST(OsiGroundTruthHandler, EgoIdentifiedByHostVehicleId)
{
  EntityIdRegistry build_registry;
  auto gt = makeSimpleGroundTruth(build_registry);

  OsiGroundTruthHandler handler;
  handler.getRegistry().assign("ego");
  handler.getRegistry().assign("npc_1");

  auto frame = handler.parseGroundTruth(gt);

  // The entity matching host_vehicle_id should be marked as EGO
  bool ego_found = false;
  for (const auto & entity : frame.moving_entities) {
    if (entity.name == frame.ego_name) {
      EXPECT_EQ(entity.type, EntityType::EGO);
      ego_found = true;
    }
  }
  EXPECT_TRUE(ego_found);
}
