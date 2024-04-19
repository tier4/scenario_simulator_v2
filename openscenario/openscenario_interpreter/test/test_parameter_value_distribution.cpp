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

void checkParameterValueDistribution(const std::string path, const ParameterDistribution & expected)
{
  OpenScenario script{path};
  auto actual = script.category.as<ParameterValueDistribution>().derive();
  EXPECT_EQ(expected.size(), actual.size());

  for (std::size_t i = 0; i < expected.size(); i++) {
    const auto & expected_parameter_list = expected.at(i);
    const auto & actual_parameter_list = actual.at(i);
    EXPECT_EQ(expected_parameter_list->size(), actual_parameter_list->size());
    for (const auto & [expected_parameter_name, expected_parameter] : *expected_parameter_list) {
      const auto & actual_parameter = actual_parameter_list->at(expected_parameter_name);
      using openscenario_interpreter::Boolean;
      using openscenario_interpreter::Double;
      using openscenario_interpreter::Integer;
      using openscenario_interpreter::String;
      using openscenario_interpreter::UnsignedInt;
      using openscenario_interpreter::UnsignedShort;
      if (expected_parameter.is<Integer>()) {
        EXPECT_EQ(expected_parameter.as<Integer>(), actual_parameter.as<Integer>());
      } else if (expected_parameter.is<UnsignedInt>()) {
        EXPECT_EQ(expected_parameter.as<UnsignedInt>(), actual_parameter.as<UnsignedInt>());
      } else if (expected_parameter.is<UnsignedShort>()) {
        EXPECT_EQ(expected_parameter.as<UnsignedShort>(), actual_parameter.as<UnsignedShort>());
      } else if (expected_parameter.is<String>()) {
        EXPECT_EQ(expected_parameter.as<String>(), actual_parameter.as<String>());
      } else if (expected_parameter.is<Double>()) {
        EXPECT_EQ(expected_parameter.as<Double>(), actual_parameter.as<Double>());
      } else if (expected_parameter.is<Boolean>()) {
        EXPECT_EQ(expected_parameter.as<Boolean>(), actual_parameter.as<Boolean>());
      } else {
        THROW_SYNTAX_ERROR(std::quoted(expected_parameter_name), "is unexpected type");
      }
    }
  }
}

template <typename T, typename U>
auto makeParameterListSharedPtr(const std::string & name, U value)
{
  auto parameter_list = std::make_shared<openscenario_interpreter::ParameterList>();
  (*parameter_list)[name] = openscenario_interpreter::make<T>(value);
  return parameter_list;
}

TEST(ParameterValueDistribution, DistributionRange)
{
  using ament_index_cpp::get_package_share_directory;
  using openscenario_interpreter::Double;

  std::string path = get_package_share_directory("openscenario_interpreter") +
                     "/test/parameter_value_distribution/Deterministic.DistributionRange.xosc";

  ParameterDistribution expected_distribution;
  expected_distribution.push_back(makeParameterListSharedPtr<Double>("offset", -1.0));
  expected_distribution.push_back(makeParameterListSharedPtr<Double>("offset", 0.0));
  expected_distribution.push_back(makeParameterListSharedPtr<Double>("offset", 1.0));

  checkParameterValueDistribution(path, expected_distribution);
}

TEST(ParameterValueDistribution, DistributionSet)
{
  using ament_index_cpp::get_package_share_directory;
  using openscenario_interpreter::String;

  std::string path = get_package_share_directory("openscenario_interpreter") +
                     "/test/parameter_value_distribution/Deterministic.DistributionSet.xosc";

  ParameterDistribution expected_distribution;
  expected_distribution.push_back(makeParameterListSharedPtr<String>("LANE_ID", "34510"));
  expected_distribution.push_back(makeParameterListSharedPtr<String>("LANE_ID", "34513"));
  expected_distribution.push_back(makeParameterListSharedPtr<String>("LANE_ID", "34564"));

  checkParameterValueDistribution(path, expected_distribution);
}

TEST(ParameterValueDistribution, ValueSetDistribution)
{
  using ament_index_cpp::get_package_share_directory;
  using openscenario_interpreter::make;
  using openscenario_interpreter::String;

  std::string path = get_package_share_directory("openscenario_interpreter") +
                     "/test/parameter_value_distribution/Deterministic.ValueSetDistribution.xosc";

  ParameterDistribution expected_distribution;
  auto parameter_list = std::make_shared<openscenario_interpreter::ParameterList>();
  (*parameter_list)["LANE_ID"] = make<String>("34564");
  (*parameter_list)["offset"] = make<String>("1.0");
  expected_distribution.push_back(parameter_list);

  checkParameterValueDistribution(path, expected_distribution);
}

// Histogram
TEST(ParameterValueDistribution, Histogram)
{
  using ament_index_cpp::get_package_share_directory;
  using openscenario_interpreter::Double;

  std::string path = get_package_share_directory("openscenario_interpreter") +
                     "/test/parameter_value_distribution/Stochastic.Histogram.xosc";

  ParameterDistribution expected_distribution;
  expected_distribution.push_back(makeParameterListSharedPtr<Double>("offset", -0.657155383483317367954157361964));
  expected_distribution.push_back(makeParameterListSharedPtr<Double>("offset", -0.122937022973606868703200234449));
  expected_distribution.push_back(makeParameterListSharedPtr<Double>("offset", -0.068217520040680490467366325902));

  checkParameterValueDistribution(path, expected_distribution);
}

TEST(ParameterValueDistribution, LogNormalDistribution)
{
  using ament_index_cpp::get_package_share_directory;
  using openscenario_interpreter::Double;

  std::string path = get_package_share_directory("openscenario_interpreter") +
                     "/test/parameter_value_distribution/Stochastic.LogNormalDistribution.xosc";

  ParameterDistribution expected_distribution;
  expected_distribution.push_back(makeParameterListSharedPtr<Double>("offset", 0.0));
  expected_distribution.push_back(makeParameterListSharedPtr<Double>("offset", 0.0));
  expected_distribution.push_back(makeParameterListSharedPtr<Double>("offset", 0.0));

  checkParameterValueDistribution(path, expected_distribution);
}

TEST(ParameterValueDistribution, NormalDistribution)
{
  using ament_index_cpp::get_package_share_directory;
  using openscenario_interpreter::Double;

  std::string path = get_package_share_directory("openscenario_interpreter") +
                     "/test/parameter_value_distribution/Stochastic.NormalDistribution.xosc";

  ParameterDistribution expected_distribution;
  expected_distribution.push_back(makeParameterListSharedPtr<Double>("offset", 0.793935916513793027426970638771));
  expected_distribution.push_back(makeParameterListSharedPtr<Double>("offset", 0.214115627062221924870044631461));
  expected_distribution.push_back(makeParameterListSharedPtr<Double>("offset", 0.050105047474801357731966078290));

  checkParameterValueDistribution(path, expected_distribution);
}

// PoissonDistribution
TEST(ParameterValueDistribution, PoissonDistribution)
{
  using ament_index_cpp::get_package_share_directory;
  using openscenario_interpreter::Double;
  using openscenario_interpreter::make;
  using openscenario_interpreter::String;

  std::string path = get_package_share_directory("openscenario_interpreter") +
                     "/test/parameter_value_distribution/Stochastic.PoissonDistribution.xosc";

  ParameterDistribution expected_distribution;
  expected_distribution.push_back(makeParameterListSharedPtr<Double>("offset", 0.0));
  expected_distribution.push_back(makeParameterListSharedPtr<Double>("offset", 1.0));
  expected_distribution.push_back(makeParameterListSharedPtr<Double>("offset", 1.0));

  checkParameterValueDistribution(path, expected_distribution);
}

// ProbabilityDistribution
TEST(ParameterValueDistribution, ProbabilityDistributionSet)
{
  using ament_index_cpp::get_package_share_directory;
  using openscenario_interpreter::String;

  std::string path = get_package_share_directory("openscenario_interpreter") +
                     "/test/parameter_value_distribution/Stochastic.ProbabilityDistributionSet.xosc";

  ParameterDistribution expected_distribution;
  expected_distribution.push_back(makeParameterListSharedPtr<String>("offset", "0.0"));
  expected_distribution.push_back(makeParameterListSharedPtr<String>("offset", "1.0"));
  expected_distribution.push_back(makeParameterListSharedPtr<String>("offset", "1.0"));

  checkParameterValueDistribution(path, expected_distribution);
}

// UniformDistribution
TEST(ParameterValueDistribution, UniformDistribution)
{
  using ament_index_cpp::get_package_share_directory;
  using openscenario_interpreter::Double;

  std::string path = get_package_share_directory("openscenario_interpreter") +
                     "/test/parameter_value_distribution/Stochastic.UniformDistribution.xosc";

  ParameterDistribution expected_distribution;
  expected_distribution.push_back(makeParameterListSharedPtr<Double>("offset", 0.185689233033365264091685276071));
  expected_distribution.push_back(makeParameterListSharedPtr<Double>("offset", 0.688531488513196565648399882775));
  expected_distribution.push_back(makeParameterListSharedPtr<Double>("offset", 0.715891239979659754766316837049));

  checkParameterValueDistribution(path, expected_distribution);
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
