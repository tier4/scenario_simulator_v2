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
#include <boost/filesystem.hpp>
#include <openscenario_interpreter/syntax/open_scenario.hpp>

using ament_index_cpp::get_package_share_directory;
using openscenario_interpreter::OpenScenario;

std::string getTestFilePath(const std::string & filename)
{
  return get_package_share_directory("openscenario_interpreter") +
         "/test/parameter_value_distribution/" + filename;
}

class ParameterValueDistributionTest : public testing::TestWithParam<std::string>
{
};

TEST_P(ParameterValueDistributionTest, ParseXoscFile)
{
  auto path = getTestFilePath(GetParam());
  ASSERT_TRUE(boost::filesystem::exists(path)) << "File not found: " << path;
  EXPECT_NO_THROW(OpenScenario script{path});
}

// clang-format off
INSTANTIATE_TEST_SUITE_P(
  Stochastic, ParameterValueDistributionTest,
  testing::Values(
    "Stochastic.Histogram.xosc",
    "Stochastic.NormalDistribution.xosc",
    "Stochastic.PoissonDistribution.xosc",
    "Stochastic.ProbabilityDistributionSet.xosc",
    "Stochastic.UniformDistribution.xosc"));
// clang-format on

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
