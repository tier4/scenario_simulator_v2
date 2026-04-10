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

#include <osi_interface/traffic_update_builder.hpp>

using namespace osi_interface;

constexpr double kTol = 1e-9;

TEST(TrafficUpdateBuilder, BasicConstruction)
{
  EntityIdRegistry registry;
  auto tu = TrafficUpdateBuilder(registry).setTimestamp(3.0).build();

  EXPECT_EQ(tu.version().version_major(), 3u);
  EXPECT_EQ(tu.version().version_minor(), 8u);
  EXPECT_NEAR(fromOsiTimestamp(tu.timestamp()), 3.0, kTol);
  EXPECT_EQ(tu.update_size(), 0);
  EXPECT_EQ(tu.internal_state_size(), 0);
}

TEST(TrafficUpdateBuilder, EgoUpdate)
{
  EntityIdRegistry registry;

  EntityData ego;
  ego.name = "ego";
  ego.type = EntityType::EGO;
  ego.subtype = EntitySubtype::CAR;
  ego.pose = {10.0, 20.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  ego.twist = {5.0, 0.0, 0.0, 0.0, 0.0, 0.1};
  ego.accel = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  ego.bounding_box = {0.0, 0.0, 0.0, 4.5, 1.8, 1.5};

  auto tu = TrafficUpdateBuilder(registry).setTimestamp(1.0).setEgoUpdate(ego).build();

  ASSERT_EQ(tu.update_size(), 1);
  const auto & obj = tu.update(0);
  EXPECT_TRUE(obj.has_id());
  EXPECT_NEAR(obj.base().position().x(), 10.0, kTol);
  EXPECT_NEAR(obj.base().velocity().x(), 5.0, kTol);
}

TEST(TrafficUpdateBuilder, WithHostVehicleData)
{
  EntityIdRegistry registry;

  EntityData ego;
  ego.name = "ego";
  ego.type = EntityType::EGO;
  ego.subtype = EntitySubtype::CAR;
  ego.pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  ego.bounding_box = {0.0, 0.0, 0.0, 4.0, 2.0, 1.5};

  osi3::HostVehicleData hvd;
  hvd.mutable_vehicle_motion()->mutable_velocity()->set_x(10.0);

  auto tu = TrafficUpdateBuilder(registry)
              .setTimestamp(2.0)
              .setEgoUpdate(ego)
              .setHostVehicleData(hvd)
              .build();

  ASSERT_EQ(tu.update_size(), 1);
  ASSERT_EQ(tu.internal_state_size(), 1);
  EXPECT_NEAR(tu.internal_state(0).vehicle_motion().velocity().x(), 10.0, kTol);
}

TEST(TrafficUpdateBuilder, TimestampMatchesGroundTruth)
{
  EntityIdRegistry registry;

  // Build a GroundTruth-like timestamp
  double sim_time = 42.123;

  auto tu = TrafficUpdateBuilder(registry).setTimestamp(sim_time).build();

  // TrafficUpdate timestamp should match (no latency for TPM)
  EXPECT_NEAR(fromOsiTimestamp(tu.timestamp()), sim_time, kTol);
}

TEST(TrafficUpdateBuilder, EgoIdConsistentWithRegistry)
{
  EntityIdRegistry registry;

  // Assign ego ID first (simulating what GroundTruthBuilder does)
  auto ego_id = registry.assign("ego");

  EntityData ego;
  ego.name = "ego";
  ego.type = EntityType::EGO;
  ego.subtype = EntitySubtype::CAR;
  ego.pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  ego.bounding_box = {0.0, 0.0, 0.0, 4.0, 2.0, 1.5};

  auto tu = TrafficUpdateBuilder(registry).setTimestamp(1.0).setEgoUpdate(ego).build();

  ASSERT_EQ(tu.update_size(), 1);
  // ID should be the same as what was assigned by the registry
  EXPECT_EQ(tu.update(0).id().value(), ego_id.value());
}
