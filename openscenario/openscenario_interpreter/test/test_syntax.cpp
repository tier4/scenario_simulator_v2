// Copyright 2015 TIER IV, Inc. All rights reserved.
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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <openscenario_interpreter/syntax/open_scenario.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

TEST(syntax, dummy) { ASSERT_TRUE(true); }

// TEST(Syntax, LexicalScope)
// {
//   using ament_index_cpp::get_package_share_directory;
//
//   auto node {
//     std::make_shared<rclcpp::Node>("", rclcpp::NodeOptions())
//   };
//
//   openscenario_interpreter::OpenScenario interpreter {
//     get_package_share_directory("openscenario_interpreter") + "/test/lexical-scope.xosc",
//     node
//   };
// }
//
// TEST(Error, Success)
// {
//   using ament_index_cpp::get_package_share_directory;
//
//   auto node {
//     std::make_shared<rclcpp::Node>("", rclcpp::NodeOptions())
//   };
//
//   openscenario_interpreter::OpenScenario evaluate {
//     get_package_share_directory("openscenario_interpreter") + "/test/success.xosc",
//     node
//   };
//
//   ASSERT_FALSE(evaluate.complete());
//
//   const auto begin {std::chrono::high_resolution_clock::now()};
//
//   using std::chrono_literals::operator""ms;
//
//   rclcpp::WallRate rate {50ms};
//
//   using openscenario_interpreter::complete_state;
//
//   for (evaluate.init(); evaluate() != complete_state; rate.sleep()) {
//     ASSERT_LT(
//       std::chrono::duration_cast<std::chrono::seconds>(
//         std::chrono::high_resolution_clock::now() - begin).count(),
//       20);
//   }
// }

// TEST(Syntax, invalid)
// {
//   auto f = []()
//   {
//     openscenario_interpreter::OpenSCENARIO osc { XOSC("invalid-1.xosc") };
//   };
//
//   EXPECT_THROW({ f(); }, openscenario_interpreter::SyntaxError);
// }

// TEST(Syntax, scenarioDefinition)
// {
//   using namespace openscenario_interpreter;
//
//   OpenSCENARIO osc { XOSC("example.xosc"), "127.0.0.1", 5555 };
//
//   // EXPECT_TRUE(osc.element("FileHeader"));
//   // EXPECT_TRUE(osc.element("ParameterDeclarations"));
//
//   // EXPECT_EQ(osc["FileHeader"].revMajor(), 1);
//   // EXPECT_TRUE(osc.catalog_locations);
//   // EXPECT_TRUE(osc.entities);
//   // EXPECT_TRUE(osc.parameter_declarations);
//   // EXPECT_TRUE(osc.road_network);
//   // EXPECT_TRUE(osc.storyboard);
//
//   EXPECT_TRUE(osc.evaluate().is<Boolean>());
// }

// TEST(Core, objectBinder)
// {
//   using openscenario_interpreter::make;
//
//   const auto foo {make<Double>(3.14)};
//
//   const auto result {foo.evaluate()};
//
//   EXPECT_TRUE(result.is<Double>());
//   EXPECT_TRUE(result.as<Double>().data = 3.14);
// }

// TEST(Scenario, LaneChange)
// {
//   using namespace openscenario_interpreter;
//
//   OpenSCENARIO osc { XOSC("lane_change.xosc"), "127.0.0.1", 5555 };
// }

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
