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

#include <osi_interface/osi_traffic_light_conversions.hpp>

using namespace osi_interface;
using C = TrafficLightBulb::Color;
using S = TrafficLightBulb::Shape;
using St = TrafficLightBulb::Status;

TEST(TrafficLightConversions, ColorMapping)
{
  using OC = osi3::TrafficLight::Classification;
  EXPECT_EQ(toOsiColor(C::RED), OC::COLOR_RED);
  EXPECT_EQ(toOsiColor(C::AMBER), OC::COLOR_YELLOW);
  EXPECT_EQ(toOsiColor(C::GREEN), OC::COLOR_GREEN);
  EXPECT_EQ(toOsiColor(C::WHITE), OC::COLOR_WHITE);
  EXPECT_EQ(toOsiColor(C::UNKNOWN), OC::COLOR_UNKNOWN);

  EXPECT_EQ(fromOsiColor(OC::COLOR_RED), C::RED);
  EXPECT_EQ(fromOsiColor(OC::COLOR_YELLOW), C::AMBER);
  EXPECT_EQ(fromOsiColor(OC::COLOR_GREEN), C::GREEN);
  EXPECT_EQ(fromOsiColor(OC::COLOR_WHITE), C::WHITE);
  EXPECT_EQ(fromOsiColor(OC::COLOR_UNKNOWN), C::UNKNOWN);
}

TEST(TrafficLightConversions, ShapeIconMapping)
{
  using OI = osi3::TrafficLight::Classification;
  EXPECT_EQ(toOsiIcon(S::CIRCLE), OI::ICON_NONE);
  EXPECT_EQ(toOsiIcon(S::LEFT_ARROW), OI::ICON_ARROW_LEFT);
  EXPECT_EQ(toOsiIcon(S::RIGHT_ARROW), OI::ICON_ARROW_RIGHT);
  EXPECT_EQ(toOsiIcon(S::UP_ARROW), OI::ICON_ARROW_STRAIGHT_AHEAD);
  EXPECT_EQ(toOsiIcon(S::UP_LEFT_ARROW), OI::ICON_ARROW_STRAIGHT_AHEAD_LEFT);
  EXPECT_EQ(toOsiIcon(S::UP_RIGHT_ARROW), OI::ICON_ARROW_STRAIGHT_AHEAD_RIGHT);
  EXPECT_EQ(toOsiIcon(S::UNKNOWN), OI::ICON_UNKNOWN);

  EXPECT_EQ(fromOsiIcon(OI::ICON_NONE), S::CIRCLE);
  EXPECT_EQ(fromOsiIcon(OI::ICON_ARROW_LEFT), S::LEFT_ARROW);
  EXPECT_EQ(fromOsiIcon(OI::ICON_ARROW_RIGHT), S::RIGHT_ARROW);
  EXPECT_EQ(fromOsiIcon(OI::ICON_ARROW_STRAIGHT_AHEAD), S::UP_ARROW);
  EXPECT_EQ(fromOsiIcon(OI::ICON_UNKNOWN), S::UNKNOWN);
}

TEST(TrafficLightConversions, StatusModeMapping)
{
  using OM = osi3::TrafficLight::Classification;
  EXPECT_EQ(toOsiMode(St::SOLID_OFF), OM::MODE_OFF);
  EXPECT_EQ(toOsiMode(St::SOLID_ON), OM::MODE_CONSTANT);
  EXPECT_EQ(toOsiMode(St::FLASHING), OM::MODE_FLASHING);
  EXPECT_EQ(toOsiMode(St::UNKNOWN), OM::MODE_UNKNOWN);

  EXPECT_EQ(fromOsiMode(OM::MODE_OFF), St::SOLID_OFF);
  EXPECT_EQ(fromOsiMode(OM::MODE_CONSTANT), St::SOLID_ON);
  EXPECT_EQ(fromOsiMode(OM::MODE_FLASHING), St::FLASHING);
  EXPECT_EQ(fromOsiMode(OM::MODE_UNKNOWN), St::UNKNOWN);
}

TEST(TrafficLightConversions, SignalGroupToOsiAndBack)
{
  EntityIdRegistry registry;

  TrafficSignalGroup signal;
  signal.lanelet_id = 12345;
  signal.relation_ids = {100, 200};

  TrafficLightBulb red_circle;
  red_circle.color = C::RED;
  red_circle.shape = S::CIRCLE;
  red_circle.status = St::SOLID_ON;
  signal.bulbs.push_back(red_circle);

  TrafficLightBulb green_arrow;
  green_arrow.color = C::GREEN;
  green_arrow.shape = S::LEFT_ARROW;
  green_arrow.status = St::SOLID_ON;
  signal.bulbs.push_back(green_arrow);

  auto osi_lights = toOsiTrafficLights(signal, registry);
  ASSERT_EQ(osi_lights.size(), 2u);

  // Check first bulb (red circle)
  EXPECT_TRUE(osi_lights[0].has_classification());
  EXPECT_EQ(osi_lights[0].classification().color(), osi3::TrafficLight::Classification::COLOR_RED);
  EXPECT_EQ(osi_lights[0].classification().icon(), osi3::TrafficLight::Classification::ICON_NONE);
  EXPECT_EQ(
    osi_lights[0].classification().mode(), osi3::TrafficLight::Classification::MODE_CONSTANT);
  EXPECT_EQ(osi_lights[0].classification().assigned_lane_id_size(), 2);

  // Check second bulb (green left arrow)
  EXPECT_EQ(
    osi_lights[1].classification().color(), osi3::TrafficLight::Classification::COLOR_GREEN);
  EXPECT_EQ(
    osi_lights[1].classification().icon(), osi3::TrafficLight::Classification::ICON_ARROW_LEFT);

  // Source reference
  ASSERT_GE(osi_lights[0].source_reference_size(), 1);
  EXPECT_EQ(osi_lights[0].source_reference(0).type(), "net.asam.lanelet2");
  EXPECT_EQ(osi_lights[0].source_reference(0).identifier(0), "12345");

  // Unique IDs
  EXPECT_NE(osi_lights[0].id().value(), osi_lights[1].id().value());

  // Round trip
  auto result = fromOsiTrafficLights(osi_lights, registry);
  EXPECT_EQ(result.lanelet_id, 12345);
  ASSERT_EQ(result.bulbs.size(), 2u);
  EXPECT_EQ(result.bulbs[0].color, C::RED);
  EXPECT_EQ(result.bulbs[0].shape, S::CIRCLE);
  EXPECT_EQ(result.bulbs[0].status, St::SOLID_ON);
  EXPECT_EQ(result.bulbs[1].color, C::GREEN);
  EXPECT_EQ(result.bulbs[1].shape, S::LEFT_ARROW);
  ASSERT_EQ(result.relation_ids.size(), 2u);
  EXPECT_EQ(result.relation_ids[0], 100);
  EXPECT_EQ(result.relation_ids[1], 200);
}

TEST(TrafficLightConversions, EmptySignalGroup)
{
  EntityIdRegistry registry;
  TrafficSignalGroup signal;
  signal.lanelet_id = 1;

  auto osi_lights = toOsiTrafficLights(signal, registry);
  EXPECT_TRUE(osi_lights.empty());
}
