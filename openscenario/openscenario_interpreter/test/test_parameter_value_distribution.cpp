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
#include <openscenario_interpreter/parameter_distribution.hpp>
#include <openscenario_interpreter/syntax/open_scenario.hpp>
#include <openscenario_interpreter/syntax/parameter_value_distribution_definition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

TEST(syntax, dummy) { ASSERT_TRUE(true); }

using openscenario_interpreter::OpenScenario;
using openscenario_interpreter::ParameterDistribution;
using openscenario_interpreter::ParameterValueDistribution;
using openscenario_interpreter::ParameterValueDistributionDefinition;

class ParameterValueDistributionTester
{
public:
  explicit ParameterValueDistributionTester(std::string path) : script{path}
  {
    OpenScenario script{path};
  }
  void check(const ParameterDistribution & expected)
  {
    auto & parameter_value_distribution = script.category.as<ParameterValueDistribution>();
    ParameterDistribution derived_distribution = parameter_value_distribution.derive();
    EXPECT_EQ(expected.size(), derived_distribution.size());

    for (std::size_t i = 0; i < expected.size(); i++) {
      const auto & expected_parameter_list = expected.at(i);
      const auto & derived_parameter_list = derived_distribution.at(i);
      EXPECT_EQ(expected_parameter_list->size(), derived_parameter_list->size());
      for (const auto & expected_parameter : *expected_parameter_list) {
        const auto & derived_parameter = derived_parameter_list->at(expected_parameter.first);
        EXPECT_EQ(expected_parameter.second, derived_parameter);
      }
    }
  }

private:
  OpenScenario script;
};

TEST(ParameterValueDistribution, DistributionRange)
{
  using ament_index_cpp::get_package_share_directory;

  std::string path = get_package_share_directory("openscenario_interpreter") +
                     "/test/parameter_value_distribution/Deterministic.DistributionRange.xosc";

  ParameterValueDistributionTester tester{path};

  ParameterDistribution expected_distribution;
  {
    auto parameter_list = std::make_shared<openscenario_interpreter::ParameterList>();
    (*parameter_list)["offset"] =
      openscenario_interpreter::make<openscenario_interpreter::Double>(-1.0);
    expected_distribution.emplace_back(parameter_list);
  }
  {
    auto parameter_list = std::make_shared<openscenario_interpreter::ParameterList>();
    (*parameter_list)["offset"] =
      openscenario_interpreter::make<openscenario_interpreter::Double>(0.0);
    expected_distribution.emplace_back(parameter_list);
  }
  {
    auto parameter_list = std::make_shared<openscenario_interpreter::ParameterList>();
    (*parameter_list)["offset"] =
      openscenario_interpreter::make<openscenario_interpreter::Double>(1.0);
    expected_distribution.emplace_back(parameter_list);
  }
  tester.check(expected_distribution);
}
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
