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

#include <osi_interface/entity_id_registry.hpp>

using osi_interface::EntityIdRegistry;

TEST(EntityIdRegistry, AssignReturnsUniqueIds)
{
  EntityIdRegistry registry;
  auto id_a = registry.assign("entity_a");
  auto id_b = registry.assign("entity_b");
  EXPECT_NE(id_a.value(), id_b.value());
  EXPECT_GE(id_a.value(), 1u);
  EXPECT_GE(id_b.value(), 1u);
}

TEST(EntityIdRegistry, AssignReturnsSameIdForSameName)
{
  EntityIdRegistry registry;
  auto id1 = registry.assign("ego");
  auto id2 = registry.assign("ego");
  EXPECT_EQ(id1.value(), id2.value());
}

TEST(EntityIdRegistry, LookupFindsAssignedId)
{
  EntityIdRegistry registry;
  auto assigned = registry.assign("npc_vehicle");
  auto found = registry.lookup("npc_vehicle");
  ASSERT_TRUE(found.has_value());
  EXPECT_EQ(found->value(), assigned.value());
}

TEST(EntityIdRegistry, LookupReturnsNulloptForUnknown)
{
  EntityIdRegistry registry;
  EXPECT_FALSE(registry.lookup("nonexistent").has_value());
}

TEST(EntityIdRegistry, ReverseLookupFindsName)
{
  EntityIdRegistry registry;
  auto id = registry.assign("pedestrian_1");
  auto name = registry.reverseLookup(id.value());
  ASSERT_TRUE(name.has_value());
  EXPECT_EQ(*name, "pedestrian_1");
}

TEST(EntityIdRegistry, ReverseLookupReturnsNulloptForUnknown)
{
  EntityIdRegistry registry;
  EXPECT_FALSE(registry.reverseLookup(999).has_value());
}

TEST(EntityIdRegistry, RemoveDeletesMapping)
{
  EntityIdRegistry registry;
  auto id = registry.assign("to_remove");
  EXPECT_TRUE(registry.remove("to_remove"));
  EXPECT_FALSE(registry.lookup("to_remove").has_value());
  EXPECT_FALSE(registry.reverseLookup(id.value()).has_value());
}

TEST(EntityIdRegistry, RemoveReturnsFalseForUnknown)
{
  EntityIdRegistry registry;
  EXPECT_FALSE(registry.remove("nonexistent"));
}

TEST(EntityIdRegistry, RemovedIdIsNotReused)
{
  EntityIdRegistry registry;
  auto id1 = registry.assign("first");
  registry.remove("first");
  auto id2 = registry.assign("second");
  EXPECT_NE(id1.value(), id2.value());
}

TEST(EntityIdRegistry, ClearResetsAll)
{
  EntityIdRegistry registry;
  registry.assign("a");
  registry.assign("b");
  EXPECT_EQ(registry.size(), 2u);
  registry.clear();
  EXPECT_EQ(registry.size(), 0u);
  EXPECT_FALSE(registry.lookup("a").has_value());
}

TEST(EntityIdRegistry, SizeTracksEntities)
{
  EntityIdRegistry registry;
  EXPECT_EQ(registry.size(), 0u);
  registry.assign("x");
  EXPECT_EQ(registry.size(), 1u);
  registry.assign("y");
  EXPECT_EQ(registry.size(), 2u);
  registry.remove("x");
  EXPECT_EQ(registry.size(), 1u);
}
