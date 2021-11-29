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

#include <gtest/gtest.h>

#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/entity/vehicle_entity.hpp>
#include <traffic_simulator/helper/helper.hpp>

#include "../catalogs.hpp"
#include "../expect_eq_macros.hpp"

/*
TEST(VEHICLE_ENTITY, GET_VEHICLE_COMMAND)
{
  traffic_simulator::entity::VehicleEntity entity("vehicle", getVehicleParameters());
  EXPECT_THROW(entity.getVehicleCommand(), common::SimulationError);
}

TEST(VEHICLE_ENTITY, SET_STATUS)
{
  traffic_simulator::entity::VehicleEntity entity("vehicle", getVehicleParameters());
  traffic_simulator_msgs::msg::EntityStatus status;
  status.lanelet_pose = traffic_simulator::helper::constructLaneletPose(34741, 0, 0);
  status.action_status = traffic_simulator::helper::constructActionStatus(3);
  status.bounding_box = entity.getBoundingBox();
  EXPECT_NO_THROW(entity.setStatus(status));
}

TEST(VEHICLE_ENTITY, UPDATE_ENTITY_TIMESTAMP)
{
  traffic_simulator::entity::VehicleEntity entity("vehicle", getVehicleParameters());
  EXPECT_NO_THROW(entity.updateEntityStatusTimestamp(3));
  traffic_simulator_msgs::msg::EntityStatus status;
  status.lanelet_pose = traffic_simulator::helper::constructLaneletPose(34741, 0, 0);
  status.action_status = traffic_simulator::helper::constructActionStatus(3);
  status.bounding_box = entity.getBoundingBox();
  EXPECT_NO_THROW(entity.setStatus(status));
  EXPECT_DOUBLE_EQ(entity.getStatus().time, 0.0);
  EXPECT_NO_THROW(entity.updateEntityStatusTimestamp(3));
  EXPECT_DOUBLE_EQ(entity.getStatus().time, 3.0);
}
*/

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
