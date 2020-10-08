// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <scenario_runner/syntax/open_scenario.hpp>

#include <chrono>
#include <cstdlib>
#include <thread>

TEST(syntax, LexicalScope)
{
  scenario_runner::OpenScenario interperter {
    ament_index_cpp::get_package_share_directory("scenario_runner") + "/test/lexical-scope.xosc",
    "127.0.0.1",
    8080
  };
}

TEST(error, Success)
{
  scenario_runner::OpenScenario evaluate {
    ament_index_cpp::get_package_share_directory("scenario_runner") + "/test/success.xosc",
    "127.0.0.1",
    8080
  };

  ASSERT_FALSE(evaluate.complete());

  const auto begin {std::chrono::high_resolution_clock::now()};

  using std::chrono_literals::operator""ms;

  rclcpp::WallRate rate {50ms};

  using scenario_runner::complete_state;

  for (evaluate.init(); evaluate() != complete_state; rate.sleep()) {
    ASSERT_LT(
      std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::high_resolution_clock::now() - begin).count(),
      20);
  }
}

// TEST(Syntax, invalid)
// {
//   auto f = []()
//   {
//     scenario_runner::OpenSCENARIO osc { XOSC("invalid-1.xosc") };
//   };
//
//   EXPECT_THROW({ f(); }, scenario_runner::SyntaxError);
// }

// TEST(Syntax, scenarioDefinition)
// {
//   using namespace scenario_runner;
//
//   OpenSCENARIO osc { XOSC("example.xosc"), "127.0.0.1", 8080 };
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
//   using scenario_runner::make;
//
//   const auto hoge {make<Double>(3.14)};
//
//   const auto result {hoge.evaluate()};
//
//   EXPECT_TRUE(result.is<Double>());
//   EXPECT_TRUE(result.as<Double>().data = 3.14);
// }

// TEST(Scenario, LaneChange)
// {
//   using namespace scenario_runner;
//
//   OpenSCENARIO osc { XOSC("lane_change.xosc"), "127.0.0.1", 8080 };
// }

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
