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

#include <chrono>
#include <cmath>
#include <osi_interface/ground_truth_builder.hpp>
#include <osi_interface/osi_entity_conversions.hpp>
#include <osi_interface/osi_ground_truth_handler.hpp>
#include <osi_interface/osi_traffic_light_conversions.hpp>
#include <osi_interface/osi_zmq_client.hpp>
#include <osi_interface/osi_zmq_server.hpp>
#include <osi_interface/traffic_update_builder.hpp>
#include <thread>

using namespace osi_interface;

// Tolerance for Quaternion↔Euler round-trip (main source of precision loss)
constexpr double kQuatEulerTol = 1e-10;
// Tolerance for direct value pass-through
constexpr double kDirectTol = 1e-12;

// ============================================================================
// Test 1: EntityData → GroundTruth → parse → EntityData round-trip
// ============================================================================

class EntityDataRoundTrip : public ::testing::Test
{
protected:
  EntityIdRegistry build_registry_;
  OsiGroundTruthHandler handler_;

  void SetUp() override
  {
    // Pre-populate handler's registry with same names to ensure ID consistency
    handler_.getRegistry().assign("ego");
    handler_.getRegistry().assign("npc_vehicle");
    handler_.getRegistry().assign("pedestrian_1");
    handler_.getRegistry().assign("barrier");
  }

  auto makeEgo() -> EntityData
  {
    EntityData e;
    e.name = "ego";
    e.type = EntityType::EGO;
    e.subtype = EntitySubtype::CAR;
    e.pose = {100.5, 200.3, 0.1, 0.0, 0.0, std::sin(0.75), std::cos(0.75)};  // yaw ≈ 1.5 rad
    e.twist = {10.0, 0.5, 0.0, 0.0, 0.0, 0.1};
    e.accel = {1.5, -0.2, 0.0, 0.0, 0.0, 0.01};
    e.bounding_box = {0.0, 0.0, 0.0, 4.5, 1.8, 1.5};
    return e;
  }

  auto makeNpc() -> EntityData
  {
    EntityData e;
    e.name = "npc_vehicle";
    e.type = EntityType::VEHICLE;
    e.subtype = EntitySubtype::TRUCK;
    e.pose = {50.0, 60.0, 0.0, 0.0, 0.0, std::sin(0.5), std::cos(0.5)};
    e.twist = {15.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    e.accel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    e.bounding_box = {0.0, 0.0, 0.0, 8.0, 2.5, 3.0};
    return e;
  }

  auto makePedestrian() -> EntityData
  {
    EntityData e;
    e.name = "pedestrian_1";
    e.type = EntityType::PEDESTRIAN;
    e.subtype = EntitySubtype::PEDESTRIAN;
    e.pose = {30.0, 40.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    e.twist = {1.2, 0.0, 0.0, 0.0, 0.0, 0.0};
    e.bounding_box = {0.0, 0.0, 0.0, 0.5, 0.5, 1.7};
    return e;
  }
};

TEST_F(EntityDataRoundTrip, PosePreservedThroughGroundTruth)
{
  auto ego = makeEgo();
  auto npc = makeNpc();
  auto ped = makePedestrian();

  auto gt = GroundTruthBuilder(build_registry_)
              .setTimestamp(1.0)
              .setHostVehicle("ego")
              .setMovingEntities({ego, npc, ped})
              .build();

  auto frame = handler_.parseGroundTruth(gt);

  ASSERT_EQ(frame.moving_entities.size(), 3u);

  // Find ego in parsed results
  const EntityData * parsed_ego = nullptr;
  const EntityData * parsed_npc = nullptr;
  const EntityData * parsed_ped = nullptr;
  for (const auto & e : frame.moving_entities) {
    if (e.name == "ego") parsed_ego = &e;
    if (e.name == "npc_vehicle") parsed_npc = &e;
    if (e.name == "pedestrian_1") parsed_ped = &e;
  }
  ASSERT_NE(parsed_ego, nullptr);
  ASSERT_NE(parsed_npc, nullptr);
  ASSERT_NE(parsed_ped, nullptr);

  // Position: direct pass-through (no conversion)
  EXPECT_NEAR(parsed_ego->pose.x, ego.pose.x, kDirectTol);
  EXPECT_NEAR(parsed_ego->pose.y, ego.pose.y, kDirectTol);
  EXPECT_NEAR(parsed_ego->pose.z, ego.pose.z, kDirectTol);

  // Orientation: Quaternion→Euler→Quaternion round-trip
  EXPECT_NEAR(parsed_ego->pose.qx, ego.pose.qx, kQuatEulerTol);
  EXPECT_NEAR(parsed_ego->pose.qy, ego.pose.qy, kQuatEulerTol);
  EXPECT_NEAR(parsed_ego->pose.qz, ego.pose.qz, kQuatEulerTol);
  EXPECT_NEAR(parsed_ego->pose.qw, ego.pose.qw, kQuatEulerTol);

  // Velocity: direct pass-through
  EXPECT_NEAR(parsed_ego->twist.linear_x, ego.twist.linear_x, kDirectTol);
  EXPECT_NEAR(parsed_ego->twist.linear_y, ego.twist.linear_y, kDirectTol);
  EXPECT_NEAR(parsed_ego->twist.angular_z, ego.twist.angular_z, kDirectTol);

  // Acceleration: direct pass-through
  EXPECT_NEAR(parsed_ego->accel.linear_x, ego.accel.linear_x, kDirectTol);
  EXPECT_NEAR(parsed_ego->accel.linear_y, ego.accel.linear_y, kDirectTol);

  // Bounding box: direct pass-through
  EXPECT_NEAR(parsed_ego->bounding_box.length, ego.bounding_box.length, kDirectTol);
  EXPECT_NEAR(parsed_ego->bounding_box.width, ego.bounding_box.width, kDirectTol);
  EXPECT_NEAR(parsed_ego->bounding_box.height, ego.bounding_box.height, kDirectTol);

  // NPC vehicle
  EXPECT_NEAR(parsed_npc->pose.x, npc.pose.x, kDirectTol);
  EXPECT_NEAR(parsed_npc->twist.linear_x, npc.twist.linear_x, kDirectTol);
  EXPECT_EQ(parsed_npc->type, EntityType::VEHICLE);

  // Pedestrian
  EXPECT_NEAR(parsed_ped->pose.x, ped.pose.x, kDirectTol);
  EXPECT_EQ(parsed_ped->type, EntityType::PEDESTRIAN);
}

TEST_F(EntityDataRoundTrip, TimestampPreserved)
{
  auto gt = GroundTruthBuilder(build_registry_)
              .setTimestamp(42.123456789)
              .setHostVehicle("ego")
              .setMovingEntities({makeEgo()})
              .build();

  auto frame = handler_.parseGroundTruth(gt);
  EXPECT_NEAR(frame.simulation_time, 42.123456789, 1e-6);
}

TEST_F(EntityDataRoundTrip, EntityTypePreserved)
{
  auto ego = makeEgo();
  auto npc = makeNpc();
  auto ped = makePedestrian();

  auto gt = GroundTruthBuilder(build_registry_)
              .setTimestamp(1.0)
              .setHostVehicle("ego")
              .setMovingEntities({ego, npc, ped})
              .build();

  auto frame = handler_.parseGroundTruth(gt);

  for (const auto & e : frame.moving_entities) {
    if (e.name == "ego") {
      EXPECT_EQ(e.type, EntityType::EGO);
    } else if (e.name == "npc_vehicle") {
      EXPECT_EQ(e.type, EntityType::VEHICLE);
      EXPECT_EQ(e.subtype, EntitySubtype::TRUCK);
    } else if (e.name == "pedestrian_1") {
      EXPECT_EQ(e.type, EntityType::PEDESTRIAN);
    }
  }
}

// ============================================================================
// Test 2: Full ZMQ communication round-trip (Ego echo)
// ============================================================================

TEST(FullCommunicationRoundTrip, EgoStatePreservedThroughZmq)
{
  constexpr int kPort = 15570;

  EntityIdRegistry server_registry;

  // Server: parse GT, extract ego, build TU with same ego state
  auto handler = [&server_registry](const osi3::GroundTruth & gt) -> osi3::TrafficUpdate {
    OsiGroundTruthHandler osi_handler;
    // Ensure the handler's registry matches
    for (int i = 0; i < gt.moving_object_size(); ++i) {
      const auto & obj = gt.moving_object(i);
      if (obj.source_reference_size() > 0 && obj.source_reference(0).identifier_size() > 0) {
        const auto & name = obj.source_reference(0).identifier(0);
        server_registry.assign(name);
        osi_handler.getRegistry().assign(name);
      }
    }

    auto frame = osi_handler.parseGroundTruth(gt);

    // Find ego and return as TrafficUpdate
    for (const auto & e : frame.moving_entities) {
      if (e.type == EntityType::EGO) {
        return osi_handler.buildTrafficUpdate(frame.simulation_time, e);
      }
    }
    osi3::TrafficUpdate tu;
    *tu.mutable_timestamp() = gt.timestamp();
    return tu;
  };

  OsiZmqServer server(kPort, handler);
  server.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  OsiZmqClient client("localhost", kPort);

  // Build GroundTruth with ego
  EntityIdRegistry client_registry;
  EntityData ego;
  ego.name = "ego";
  ego.type = EntityType::EGO;
  ego.subtype = EntitySubtype::CAR;
  ego.pose = {123.456, 789.012, 0.5, 0.0, 0.0, std::sin(1.0), std::cos(1.0)};
  ego.twist = {12.5, 0.3, 0.0, 0.0, 0.0, 0.05};
  ego.accel = {2.0, -0.1, 0.0, 0.0, 0.0, 0.0};
  ego.bounding_box = {0.0, 0.0, 0.0, 4.5, 1.8, 1.5};

  auto gt = GroundTruthBuilder(client_registry)
              .setTimestamp(5.0)
              .setHostVehicle("ego")
              .setMovingEntities({ego})
              .build();

  auto tu = client.sendGroundTruth(gt);

  // Verify TrafficUpdate contains ego with preserved values
  ASSERT_EQ(tu.update_size(), 1);

  // Parse the returned MovingObject
  EntityIdRegistry result_registry;
  result_registry.assign("ego");
  auto result = fromOsiMovingObject(tu.update(0), result_registry);

  // Position: should match (no BB offset since center=0)
  EXPECT_NEAR(result.pose.x, ego.pose.x, kDirectTol);
  EXPECT_NEAR(result.pose.y, ego.pose.y, kDirectTol);
  EXPECT_NEAR(result.pose.z, ego.pose.z, kDirectTol);

  // Quaternion: double round-trip (client Quat→Euler, server Euler→Quat→Euler, client Euler→Quat)
  // Expected precision is lower due to double conversion
  EXPECT_NEAR(result.pose.qx, ego.pose.qx, kQuatEulerTol);
  EXPECT_NEAR(result.pose.qy, ego.pose.qy, kQuatEulerTol);
  EXPECT_NEAR(result.pose.qz, ego.pose.qz, kQuatEulerTol);
  EXPECT_NEAR(result.pose.qw, ego.pose.qw, kQuatEulerTol);

  // Velocity
  EXPECT_NEAR(result.twist.linear_x, ego.twist.linear_x, kDirectTol);
  EXPECT_NEAR(result.twist.linear_y, ego.twist.linear_y, kDirectTol);
  EXPECT_NEAR(result.twist.angular_z, ego.twist.angular_z, kDirectTol);

  // Acceleration
  EXPECT_NEAR(result.accel.linear_x, ego.accel.linear_x, kDirectTol);

  // Timestamp
  EXPECT_NEAR(fromOsiTimestamp(tu.timestamp()), 5.0, 1e-9);

  client.close();
  server.stop();
}

// ============================================================================
// Test 3: BoundingBox center offset handling
// ============================================================================

TEST(BoundingBoxOffset, PositionAdjustedForCenterOffset)
{
  EntityIdRegistry registry;

  EntityData entity;
  entity.name = "offset_car";
  entity.type = EntityType::VEHICLE;
  entity.subtype = EntitySubtype::CAR;
  entity.pose = {100.0, 200.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  entity.bounding_box = {0.5, 0.0, 0.3, 4.5, 1.8, 1.5};  // center offset (0.5, 0, 0.3)

  auto obj = toOsiMovingObject(entity, registry);

  // OSI position = entity pose + BB center offset
  EXPECT_NEAR(obj.base().position().x(), 100.5, kDirectTol);
  EXPECT_NEAR(obj.base().position().y(), 200.0, kDirectTol);
  EXPECT_NEAR(obj.base().position().z(), 0.3, kDirectTol);

  // Dimensions should be the BB dimensions (not affected by center)
  EXPECT_NEAR(obj.base().dimension().length(), 4.5, kDirectTol);
  EXPECT_NEAR(obj.base().dimension().width(), 1.8, kDirectTol);
  EXPECT_NEAR(obj.base().dimension().height(), 1.5, kDirectTol);
}

TEST(BoundingBoxOffset, ZeroCenterOffsetPreservesPosition)
{
  EntityIdRegistry registry;

  EntityData entity;
  entity.name = "no_offset_car";
  entity.type = EntityType::VEHICLE;
  entity.subtype = EntitySubtype::CAR;
  entity.pose = {100.0, 200.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  entity.bounding_box = {0.0, 0.0, 0.0, 4.5, 1.8, 1.5};  // no offset

  auto obj = toOsiMovingObject(entity, registry);
  auto result = fromOsiMovingObject(obj, registry);

  // With zero offset, position should be perfectly preserved
  EXPECT_NEAR(result.pose.x, entity.pose.x, kDirectTol);
  EXPECT_NEAR(result.pose.y, entity.pose.y, kDirectTol);
  EXPECT_NEAR(result.pose.z, entity.pose.z, kDirectTol);
}

// ============================================================================
// Test 4: Traffic light conversion equivalence
// ============================================================================

TEST(TrafficLightEquivalence, RoundTripPreservesAllFields)
{
  EntityIdRegistry registry;

  TrafficSignalGroup signal;
  signal.lanelet_id = 12345;
  signal.relation_ids = {100, 200, 300};

  TrafficLightBulb red_on;
  red_on.color = TrafficLightBulb::Color::RED;
  red_on.shape = TrafficLightBulb::Shape::CIRCLE;
  red_on.status = TrafficLightBulb::Status::SOLID_ON;
  signal.bulbs.push_back(red_on);

  TrafficLightBulb green_arrow;
  green_arrow.color = TrafficLightBulb::Color::GREEN;
  green_arrow.shape = TrafficLightBulb::Shape::LEFT_ARROW;
  green_arrow.status = TrafficLightBulb::Status::SOLID_ON;
  signal.bulbs.push_back(green_arrow);

  TrafficLightBulb amber_flash;
  amber_flash.color = TrafficLightBulb::Color::AMBER;
  amber_flash.shape = TrafficLightBulb::Shape::CIRCLE;
  amber_flash.status = TrafficLightBulb::Status::FLASHING;
  signal.bulbs.push_back(amber_flash);

  // Convert to OSI and include in GroundTruth
  EntityIdRegistry gt_registry;
  auto gt = GroundTruthBuilder(gt_registry).setTimestamp(1.0).setTrafficSignals({signal}).build();

  // Verify GroundTruth contains the right number of traffic lights
  ASSERT_EQ(gt.traffic_light_size(), 3);

  // Parse each traffic light back
  for (int i = 0; i < gt.traffic_light_size(); ++i) {
    auto group = fromOsiTrafficLights({gt.traffic_light(i)}, gt_registry);
    ASSERT_EQ(group.bulbs.size(), 1u);

    const auto & original = signal.bulbs[i];
    const auto & parsed = group.bulbs[0];
    EXPECT_EQ(parsed.color, original.color) << "Bulb " << i << " color mismatch";
    EXPECT_EQ(parsed.shape, original.shape) << "Bulb " << i << " shape mismatch";
    EXPECT_EQ(parsed.status, original.status) << "Bulb " << i << " status mismatch";
  }

  // Verify lanelet ID preserved in source reference
  EXPECT_EQ(gt.traffic_light(0).source_reference(0).identifier(0), "12345");
}

// ============================================================================
// Test 5: Multiple arbitrary orientations round-trip
// ============================================================================

TEST(OrientationEquivalence, MultipleAnglesPreserved)
{
  EntityIdRegistry build_registry;
  OsiGroundTruthHandler handler;

  struct TestCase
  {
    double yaw;
    std::string label;
  };

  std::vector<TestCase> cases = {
    {0.0, "zero"},           {M_PI / 4.0, "45deg"}, {M_PI / 2.0, "90deg"}, {M_PI, "180deg"},
    {-M_PI / 3.0, "-60deg"}, {2.5, "2.5rad"},       {-2.5, "-2.5rad"},
  };

  for (const auto & tc : cases) {
    EntityData entity;
    entity.name = "test_" + tc.label;
    entity.type = EntityType::VEHICLE;
    entity.subtype = EntitySubtype::CAR;
    entity.pose.qz = std::sin(tc.yaw / 2.0);
    entity.pose.qw = std::cos(tc.yaw / 2.0);
    entity.bounding_box = {0.0, 0.0, 0.0, 4.0, 2.0, 1.5};

    build_registry.assign(entity.name);
    handler.getRegistry().assign(entity.name);

    auto gt =
      GroundTruthBuilder(build_registry).setTimestamp(1.0).setMovingEntities({entity}).build();

    auto frame = handler.parseGroundTruth(gt);
    ASSERT_EQ(frame.moving_entities.size(), 1u) << "Failed for " << tc.label;

    const auto & result = frame.moving_entities[0];
    EXPECT_NEAR(result.pose.qz, entity.pose.qz, kQuatEulerTol)
      << "qz mismatch for yaw=" << tc.label;
    EXPECT_NEAR(result.pose.qw, entity.pose.qw, kQuatEulerTol)
      << "qw mismatch for yaw=" << tc.label;
  }
}

// ============================================================================
// Test 6: Stationary object (MISC_OBJECT) round-trip
// ============================================================================

TEST(StationaryObjectEquivalence, MiscObjectRoundTrip)
{
  EntityIdRegistry build_registry;
  OsiGroundTruthHandler handler;
  handler.getRegistry().assign("barrier_1");

  EntityData barrier;
  barrier.name = "barrier_1";
  barrier.type = EntityType::MISC_OBJECT;
  barrier.pose = {50.0, 60.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  barrier.bounding_box = {0.0, 0.0, 0.0, 3.0, 0.5, 1.0};

  auto gt =
    GroundTruthBuilder(build_registry).setTimestamp(1.0).setStationaryEntities({barrier}).build();

  auto frame = handler.parseGroundTruth(gt);

  ASSERT_EQ(frame.stationary_entities.size(), 1u);
  const auto & result = frame.stationary_entities[0];
  EXPECT_EQ(result.name, "barrier_1");
  EXPECT_EQ(result.type, EntityType::MISC_OBJECT);
  EXPECT_NEAR(result.pose.x, barrier.pose.x, kDirectTol);
  EXPECT_NEAR(result.bounding_box.length, barrier.bounding_box.length, kDirectTol);
}
