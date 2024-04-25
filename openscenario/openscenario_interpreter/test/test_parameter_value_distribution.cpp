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
  auto parameter_set = std::make_shared<openscenario_interpreter::ParameterSet>();
  (*parameter_set)[name] = openscenario_interpreter::make<T>(value);
  return parameter_set;
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
  auto parameter_set = std::make_shared<openscenario_interpreter::ParameterSet>();
  (*parameter_set)["LANE_ID"] = make<String>("34564");
  (*parameter_set)["offset"] = make<String>("1.0");
  expected_distribution.push_back(parameter_set);

  checkParameterValueDistribution(path, expected_distribution);
}

TEST(ParameterValueDistribution, Deterministic)
{
  using ament_index_cpp::get_package_share_directory;
  using openscenario_interpreter::Double;
  using openscenario_interpreter::make;
  using openscenario_interpreter::String;

  std::string path = get_package_share_directory("openscenario_interpreter") +
                     "/test/parameter_value_distribution/Deterministic.xosc";

  ParameterDistribution expected_distribution;
  auto make_parameter_set = [](double offset, std::string lane_id) {
    auto parameter_set = std::make_shared<openscenario_interpreter::ParameterSet>();
    (*parameter_set)["offset"] = make<Double>(offset);
    (*parameter_set)["LANE_ID"] = make<String>(lane_id);
    return parameter_set;
  };
  expected_distribution.push_back(make_parameter_set(-1.0, "34510"));
  expected_distribution.push_back(make_parameter_set(-1.0, "34513"));
  expected_distribution.push_back(make_parameter_set(-1.0, "34564"));
  expected_distribution.push_back(make_parameter_set(-0.47, "34510"));
  expected_distribution.push_back(make_parameter_set(-0.47, "34513"));
  expected_distribution.push_back(make_parameter_set(-0.47, "34564"));
  expected_distribution.push_back(make_parameter_set(0.06, "34510"));
  expected_distribution.push_back(make_parameter_set(0.06, "34513"));
  expected_distribution.push_back(make_parameter_set(0.06, "34564"));
  expected_distribution.push_back(make_parameter_set(0.59, "34510"));
  expected_distribution.push_back(make_parameter_set(0.59, "34513"));
  expected_distribution.push_back(make_parameter_set(0.59, "34564"));

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
  expected_distribution.push_back(
    makeParameterListSharedPtr<Double>("offset", -1.000000000000000000000000000000));
  expected_distribution.push_back(
    makeParameterListSharedPtr<Double>("offset", -0.791349867976780196876518402860));
  expected_distribution.push_back(
    makeParameterListSharedPtr<Double>("offset", -1.000000000000000000000000000000));

  checkParameterValueDistribution(path, expected_distribution);
}

TEST(ParameterValueDistribution, LogNormalDistribution)
{
  using ament_index_cpp::get_package_share_directory;
  using openscenario_interpreter::Double;

  std::string path = get_package_share_directory("openscenario_interpreter") +
                     "/test/parameter_value_distribution/Stochastic.LogNormalDistribution.xosc";

  ParameterDistribution expected_distribution;
  expected_distribution.push_back(
    makeParameterListSharedPtr<Double>("offset", 0.679981508546370405632330857770));
  expected_distribution.push_back(
    makeParameterListSharedPtr<Double>("offset", 0.032166799749118187012886238563));
  expected_distribution.push_back(
    makeParameterListSharedPtr<Double>("offset", 0.033371713868564217841949925969));

  checkParameterValueDistribution(path, expected_distribution);
}

TEST(ParameterValueDistribution, NormalDistribution)
{
  using ament_index_cpp::get_package_share_directory;
  using openscenario_interpreter::Double;

  std::string path = get_package_share_directory("openscenario_interpreter") +
                     "/test/parameter_value_distribution/Stochastic.NormalDistribution.xosc";

  ParameterDistribution expected_distribution;
  expected_distribution.push_back(
    makeParameterListSharedPtr<Double>("offset", -0.086242833039257865701543437353));
  expected_distribution.push_back(
    makeParameterListSharedPtr<Double>("offset", -0.768496409013107117935703627154));
  expected_distribution.push_back(
    makeParameterListSharedPtr<Double>("offset", 0.483866059556305461164527059736));

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
  expected_distribution.push_back(makeParameterListSharedPtr<Double>("offset", 0.0));
  expected_distribution.push_back(makeParameterListSharedPtr<Double>("offset", 0.0));

  checkParameterValueDistribution(path, expected_distribution);
}

// ProbabilityDistribution
TEST(ParameterValueDistribution, ProbabilityDistributionSet)
{
  using ament_index_cpp::get_package_share_directory;
  using openscenario_interpreter::String;

  std::string path =
    get_package_share_directory("openscenario_interpreter") +
    "/test/parameter_value_distribution/Stochastic.ProbabilityDistributionSet.xosc";

  ParameterDistribution expected_distribution;
  expected_distribution.push_back(makeParameterListSharedPtr<String>("offset", "-1.0"));
  expected_distribution.push_back(makeParameterListSharedPtr<String>("offset", "-0.47"));
  expected_distribution.push_back(makeParameterListSharedPtr<String>("offset", "0.59"));

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
  expected_distribution.push_back(
    makeParameterListSharedPtr<Double>("offset", -0.736924424522478638266420603031));
  expected_distribution.push_back(
    makeParameterListSharedPtr<Double>("offset", -0.082699735953560393753036805720));
  expected_distribution.push_back(
    makeParameterListSharedPtr<Double>("offset", -0.562081627575042097610946711939));

  checkParameterValueDistribution(path, expected_distribution);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
