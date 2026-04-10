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

#include <osi_interface/ground_truth_builder.hpp>

using namespace osi_interface;

constexpr double kTol = 1e-9;

TEST(GroundTruthBuilder, BasicConstruction)
{
  EntityIdRegistry registry;
  auto gt = GroundTruthBuilder(registry)
              .setTimestamp(1.5)
              .setHostVehicle("ego")
              .setMapReference("/path/to/map.osm")
              .build();

  EXPECT_EQ(gt.version().version_major(), 3u);
  EXPECT_EQ(gt.version().version_minor(), 8u);
  EXPECT_EQ(gt.version().version_patch(), 0u);
  EXPECT_NEAR(fromOsiTimestamp(gt.timestamp()), 1.5, kTol);
  EXPECT_TRUE(gt.has_host_vehicle_id());
  EXPECT_EQ(gt.map_reference(), "/path/to/map.osm");
}

TEST(GroundTruthBuilder, MovingObjectsIncluded)
{
  EntityIdRegistry registry;

  std::vector<EntityData> entities;

  EntityData ego;
  ego.name = "ego";
  ego.type = EntityType::EGO;
  ego.subtype = EntitySubtype::CAR;
  ego.pose = {10.0, 20.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  ego.bounding_box = {0.0, 0.0, 0.0, 4.5, 1.8, 1.5};
  entities.push_back(ego);

  EntityData npc;
  npc.name = "npc_1";
  npc.type = EntityType::VEHICLE;
  npc.subtype = EntitySubtype::TRUCK;
  npc.pose = {30.0, 40.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  npc.bounding_box = {0.0, 0.0, 0.0, 8.0, 2.5, 3.0};
  entities.push_back(npc);

  EntityData ped;
  ped.name = "ped_1";
  ped.type = EntityType::PEDESTRIAN;
  ped.subtype = EntitySubtype::PEDESTRIAN;
  ped.pose = {50.0, 60.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  ped.bounding_box = {0.0, 0.0, 0.0, 0.5, 0.5, 1.7};
  entities.push_back(ped);

  auto gt = GroundTruthBuilder(registry)
              .setTimestamp(2.0)
              .setHostVehicle("ego")
              .setMovingEntities(entities)
              .build();

  ASSERT_EQ(gt.moving_object_size(), 3);

  // Ego should be identifiable by host_vehicle_id
  auto ego_id = gt.host_vehicle_id().value();
  bool ego_found = false;
  for (int i = 0; i < gt.moving_object_size(); ++i) {
    if (gt.moving_object(i).id().value() == ego_id) {
      ego_found = true;
      EXPECT_NEAR(gt.moving_object(i).base().position().x(), 10.0, kTol);
    }
  }
  EXPECT_TRUE(ego_found);
}

TEST(GroundTruthBuilder, StationaryObjectsIncluded)
{
  EntityIdRegistry registry;

  std::vector<EntityData> stationary;
  EntityData barrier;
  barrier.name = "barrier_1";
  barrier.type = EntityType::MISC_OBJECT;
  barrier.pose = {100.0, 200.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  barrier.bounding_box = {0.0, 0.0, 0.0, 2.0, 0.5, 1.0};
  stationary.push_back(barrier);

  auto gt = GroundTruthBuilder(registry).setStationaryEntities(stationary).build();

  ASSERT_EQ(gt.stationary_object_size(), 1);
  EXPECT_NEAR(gt.stationary_object(0).base().dimension().length(), 2.0, kTol);
}

TEST(GroundTruthBuilder, TrafficLightsIncluded)
{
  EntityIdRegistry registry;

  std::vector<TrafficSignalGroup> signals;
  TrafficSignalGroup sig;
  sig.lanelet_id = 100;
  sig.relation_ids = {200};

  TrafficLightBulb bulb;
  bulb.color = TrafficLightBulb::Color::RED;
  bulb.shape = TrafficLightBulb::Shape::CIRCLE;
  bulb.status = TrafficLightBulb::Status::SOLID_ON;
  sig.bulbs.push_back(bulb);

  TrafficLightBulb bulb2;
  bulb2.color = TrafficLightBulb::Color::GREEN;
  bulb2.shape = TrafficLightBulb::Shape::LEFT_ARROW;
  bulb2.status = TrafficLightBulb::Status::SOLID_OFF;
  sig.bulbs.push_back(bulb2);

  signals.push_back(sig);

  auto gt = GroundTruthBuilder(registry).setTrafficSignals(signals).build();

  // 1 signal group with 2 bulbs → 2 osi3::TrafficLight
  ASSERT_EQ(gt.traffic_light_size(), 2);
  EXPECT_EQ(
    gt.traffic_light(0).classification().color(), osi3::TrafficLight::Classification::COLOR_RED);
  EXPECT_EQ(
    gt.traffic_light(1).classification().color(), osi3::TrafficLight::Classification::COLOR_GREEN);
}

TEST(GroundTruthBuilder, EmptyBuild)
{
  EntityIdRegistry registry;
  auto gt = GroundTruthBuilder(registry).setTimestamp(0.0).build();

  EXPECT_EQ(gt.moving_object_size(), 0);
  EXPECT_EQ(gt.stationary_object_size(), 0);
  EXPECT_EQ(gt.traffic_light_size(), 0);
  EXPECT_EQ(gt.version().version_major(), 3u);
}

TEST(GroundTruthBuilder, HostVehicleIdMatchesMovingObject)
{
  EntityIdRegistry registry;

  std::vector<EntityData> entities;
  EntityData ego;
  ego.name = "my_ego";
  ego.type = EntityType::EGO;
  ego.subtype = EntitySubtype::CAR;
  ego.pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  ego.bounding_box = {0.0, 0.0, 0.0, 4.0, 2.0, 1.5};
  entities.push_back(ego);

  auto gt =
    GroundTruthBuilder(registry).setHostVehicle("my_ego").setMovingEntities(entities).build();

  ASSERT_EQ(gt.moving_object_size(), 1);
  EXPECT_EQ(gt.host_vehicle_id().value(), gt.moving_object(0).id().value());
}
