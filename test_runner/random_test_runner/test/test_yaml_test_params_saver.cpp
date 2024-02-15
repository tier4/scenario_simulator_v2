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
//
// Co-developed by TIER IV, Inc. and Robotec.AI sp. z o.o.

#include <gtest/gtest.h>

#include <random_test_runner/file_interactions/yaml_test_params_saver.hpp>

#include "test_utils.hpp"

std::string readFile(const std::string & path = "/tmp/result.yaml")
{
  std::ifstream file(path);
  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

void writeToFile(const std::string & data, const std::string & path = "/tmp/result.yaml")
{
  std::ofstream file;
  file.open(path);
  file << data;
  file.close();
}

// TestSuiteParameters

TEST(YamlTestParamsSaver, TestSuiteParameters_encode)
{
  YAML::convert<TestSuiteParameters> convert;
  auto rhs = TestSuiteParameters();
  auto node = convert.encode(rhs);

  EXPECT_STREQ(node["test_name"].as<std::string>().c_str(), rhs.name.c_str());
  EXPECT_STREQ(node["map_name"].as<std::string>().c_str(), rhs.map_name.c_str());
  EXPECT_DOUBLE_EQ(node["ego_goal_s"].as<double>(), rhs.ego_goal_s);
  EXPECT_EQ(node["ego_goal_lanelet_id"].as<int64_t>(), rhs.ego_goal_lanelet_id);
  EXPECT_EQ(node["ego_goal_partial_randomization"].as<bool>(), rhs.ego_goal_partial_randomization);
  EXPECT_DOUBLE_EQ(
    node["ego_goal_partial_randomization_distance"].as<double>(),
    rhs.ego_goal_partial_randomization_distance);
  EXPECT_EQ(node["npc_count"].as<int64_t>(), rhs.npcs_count);
  EXPECT_DOUBLE_EQ(node["npc_min_speed"].as<double>(), rhs.npc_min_speed);
  EXPECT_DOUBLE_EQ(node["npc_max_speed"].as<double>(), rhs.npc_max_speed);
  EXPECT_DOUBLE_EQ(
    node["npc_min_spawn_distance_from_ego"].as<double>(), rhs.npc_min_spawn_distance_from_ego);
  EXPECT_DOUBLE_EQ(
    node["npc_max_spawn_distance_from_ego"].as<double>(), rhs.npc_max_spawn_distance_from_ego);
}

TEST(YamlTestParamsSaver, TestSuiteParameters_decodeMap)
{
  YAML::convert<TestSuiteParameters> convert;

  auto rhs = TestSuiteParameters();
  auto node = YAML::Node(YAML::NodeType::Map);

  node["test_name"] = "random_test_1";
  node["map_name"] = "test_map_name";
  node["ego_goal_s"] = 15.5;
  node["ego_goal_lanelet_id"] = 1600;
  node["ego_goal_partial_randomization"] = true;
  node["ego_goal_partial_randomization_distance"] = 100.0;
  node["npc_count"] = 20;
  node["npc_min_speed"] = 0.3;
  node["npc_max_speed"] = 7.0;
  node["npc_min_spawn_distance_from_ego"] = 10.1;
  node["npc_max_spawn_distance_from_ego"] = 100.1;

  auto result = convert.decode(node, rhs);

  EXPECT_EQ(result, true);
  EXPECT_STREQ(rhs.name.c_str(), node["test_name"].as<std::string>().c_str());
  EXPECT_STREQ(rhs.map_name.c_str(), node["map_name"].as<std::string>().c_str());
  EXPECT_DOUBLE_EQ(rhs.ego_goal_s, node["ego_goal_s"].as<double>());
  EXPECT_EQ(rhs.ego_goal_lanelet_id, node["ego_goal_lanelet_id"].as<int64_t>());
  EXPECT_EQ(rhs.ego_goal_partial_randomization, node["ego_goal_partial_randomization"].as<bool>());
  EXPECT_DOUBLE_EQ(
    rhs.ego_goal_partial_randomization_distance,
    node["ego_goal_partial_randomization_distance"].as<double>());
  EXPECT_EQ(rhs.npcs_count, node["npc_count"].as<int64_t>());
  EXPECT_DOUBLE_EQ(rhs.npc_min_speed, node["npc_min_speed"].as<double>());
  EXPECT_DOUBLE_EQ(rhs.npc_max_speed, node["npc_max_speed"].as<double>());
  EXPECT_DOUBLE_EQ(
    rhs.npc_min_spawn_distance_from_ego, node["npc_min_spawn_distance_from_ego"].as<double>());
  EXPECT_DOUBLE_EQ(
    rhs.npc_max_spawn_distance_from_ego, node["npc_max_spawn_distance_from_ego"].as<double>());

  EXPECT_STREQ(rhs.name.c_str(), "random_test_1");
  EXPECT_STREQ(rhs.map_name.c_str(), "test_map_name");
  EXPECT_DOUBLE_EQ(rhs.ego_goal_s, 15.5);
  EXPECT_EQ(rhs.ego_goal_lanelet_id, 1600);
  EXPECT_EQ(rhs.ego_goal_partial_randomization, true);
  EXPECT_DOUBLE_EQ(rhs.ego_goal_partial_randomization_distance, 100.0);
  EXPECT_EQ(rhs.npcs_count, 20);
  EXPECT_DOUBLE_EQ(rhs.npc_min_speed, 0.3);
  EXPECT_DOUBLE_EQ(rhs.npc_max_speed, 7.0);
  EXPECT_DOUBLE_EQ(rhs.npc_min_spawn_distance_from_ego, 10.1);
  EXPECT_DOUBLE_EQ(rhs.npc_max_spawn_distance_from_ego, 100.1);
}

TEST(YamlTestParamsSaver, TestSuiteParameters_decodeNull)
{
  YAML::convert<TestSuiteParameters> convert;

  auto rhs = TestSuiteParameters();
  auto node = YAML::Node(YAML::NodeType::Null);
  auto result = convert.decode(node, rhs);

  EXPECT_EQ(result, false);
}

TEST(YamlTestParamsSaver, TestSuiteParameters_decodeScalar)
{
  YAML::convert<TestSuiteParameters> convert;

  auto rhs = TestSuiteParameters();
  auto node = YAML::Node(YAML::NodeType::Scalar);
  auto result = convert.decode(node, rhs);

  EXPECT_EQ(result, false);
}

TEST(YamlTestParamsSaver, TestSuiteParameters_decodeSequence)
{
  YAML::convert<TestSuiteParameters> convert;

  auto rhs = TestSuiteParameters();
  auto node = YAML::Node(YAML::NodeType::Sequence);
  auto result = convert.decode(node, rhs);

  EXPECT_EQ(result, false);
}

TEST(YamlTestParamsSaver, TestSuiteParameters_decodeUndefined)
{
  YAML::convert<TestSuiteParameters> convert;

  auto rhs = TestSuiteParameters();
  auto node = YAML::Node(YAML::NodeType::Undefined);
  auto result = convert.decode(node, rhs);

  EXPECT_EQ(result, false);
}

TEST(YamlTestParamsSaver, TestSuiteParameters_decodeDefault)
{
  YAML::convert<TestSuiteParameters> convert;

  auto rhs = TestSuiteParameters();
  auto node = YAML::Node();
  auto result = convert.decode(node, rhs);

  EXPECT_EQ(result, false);
}

// TestCaseParameters

TEST(YamlTestParamsSaver, TestCaseParameters_encode)
{
  YAML::convert<TestCaseParameters> convert;
  auto rhs = TestCaseParameters();
  auto node = convert.encode(rhs);

  EXPECT_EQ(node["seed"].as<int64_t>(), rhs.seed);
}

TEST(YamlTestParamsSaver, TestCaseParameters_decodeMap)
{
  YAML::convert<TestCaseParameters> convert;
  auto rhs = TestCaseParameters();
  auto node = YAML::Node(YAML::NodeType::Map);

  node["seed"] = 5;

  auto result = convert.decode(node, rhs);

  EXPECT_EQ(result, true);
  EXPECT_EQ(rhs.seed, 5);
}

TEST(YamlTestParamsSaver, TestCaseParameters_decodeNull)
{
  YAML::convert<TestCaseParameters> convert;
  auto rhs = TestCaseParameters();
  rhs.seed = 10;
  auto node = YAML::Node(YAML::NodeType::Null);
  auto result = convert.decode(node, rhs);

  EXPECT_EQ(result, false);
  EXPECT_EQ(rhs.seed, 10);
}

TEST(YamlTestParamsSaver, TestCaseParameters_decodeScalar)
{
  YAML::convert<TestCaseParameters> convert;
  auto rhs = TestCaseParameters();
  rhs.seed = 10;
  auto node = YAML::Node(YAML::NodeType::Scalar);
  auto result = convert.decode(node, rhs);

  EXPECT_EQ(result, false);
  EXPECT_EQ(rhs.seed, 10);
}

TEST(YamlTestParamsSaver, TestCaseParameters_decodeSequence)
{
  YAML::convert<TestCaseParameters> convert;
  auto rhs = TestCaseParameters();
  rhs.seed = 10;
  auto node = YAML::Node(YAML::NodeType::Sequence);
  auto result = convert.decode(node, rhs);

  EXPECT_EQ(result, false);
  EXPECT_EQ(rhs.seed, 10);
}

TEST(YamlTestParamsSaver, TestCaseParameters_decodeUndefined)
{
  YAML::convert<TestCaseParameters> convert;
  auto rhs = TestCaseParameters();
  rhs.seed = 10;
  auto node = YAML::Node(YAML::NodeType::Undefined);
  auto result = convert.decode(node, rhs);

  EXPECT_EQ(result, false);
  EXPECT_EQ(rhs.seed, 10);
}

TEST(YamlTestParamsSaver, TestCaseParameters_decodeDefault)
{
  YAML::convert<TestCaseParameters> convert;
  auto rhs = TestCaseParameters();
  rhs.seed = 10;
  auto node = YAML::Node();
  auto result = convert.decode(node, rhs);

  EXPECT_EQ(result, false);
  EXPECT_EQ(rhs.seed, 10);
}

TEST(YamlTestParamsSaver, YamlTestParamsIO_read)
{
  auto test_params_IO = YamlTestParamsIO(rclcpp::get_logger("yaml_test_params_saver"), "/tmp");
  std::string file_data =
    "random_test:\n\
  test_name: random_test\n\
  map_name: kashiwanoha_map\n\
  ego_goal_s: 0\n\
  ego_goal_lanelet_id: -1\n\
  ego_goal_partial_randomization: false\n\
  ego_goal_partial_randomization_distance: 25\n\
  npc_count: 10\n\
  npc_min_speed: 0.5\n\
  npc_max_speed: 3\n\
  npc_min_spawn_distance_from_ego: 10\n\
  npc_max_spawn_distance_from_ego: 100\n\
  test_cases:\n\
    - seed: 121660883\n\
    - seed: 478515236\n\
    - seed: 3404285739\n\
    - seed: 1008004056\n\
    - seed: 2015958364";
  writeToFile(file_data);
  auto result = test_params_IO.read();

  EXPECT_STREQ(result.first.name.c_str(), "random_test");
  EXPECT_STREQ(result.first.map_name.c_str(), "kashiwanoha_map");
  EXPECT_DOUBLE_EQ(result.first.ego_goal_s, 0.0);
  EXPECT_EQ(result.first.ego_goal_lanelet_id, -1);
  EXPECT_FALSE(result.first.ego_goal_partial_randomization);
  EXPECT_DOUBLE_EQ(result.first.ego_goal_partial_randomization_distance, 25.0);
  EXPECT_EQ(result.first.npcs_count, 10);
  EXPECT_DOUBLE_EQ(result.first.npc_min_speed, 0.5);
  EXPECT_DOUBLE_EQ(result.first.npc_max_speed, 3.0);
  EXPECT_DOUBLE_EQ(result.first.npc_min_spawn_distance_from_ego, 10.0);
  EXPECT_DOUBLE_EQ(result.first.npc_max_spawn_distance_from_ego, 100.0);

  EXPECT_EQ(result.second[0].seed, 121660883);
  EXPECT_EQ(result.second[1].seed, 478515236);
  EXPECT_EQ(result.second[2].seed, 3404285739);
  EXPECT_EQ(result.second[3].seed, 1008004056);
  EXPECT_EQ(result.second[4].seed, 2015958364);
}

TEST(YamlTestParamsSaver, YamlTestParamsIO_readTooManySuites)
{
  auto test_params_IO = YamlTestParamsIO(rclcpp::get_logger("yaml_test_params_saver"), "/tmp");
  std::string file_data =
    "random_test1:\n\
  test_name: random_test\n\
  map_name: kashiwanoha_map\n\
  ego_goal_s: 0\n\
  ego_goal_lanelet_id: -1\n\
  ego_goal_partial_randomization: false\n\
  ego_goal_partial_randomization_distance: 25\n\
  npc_count: 10\n\
  npc_min_speed: 0.5\n\
  npc_max_speed: 3\n\
  npc_min_spawn_distance_from_ego: 10\n\
  npc_max_spawn_distance_from_ego: 100\n\
  test_cases:\n\
    - seed: 121660883\n\
    - seed: 478515236\n\
    - seed: 3404285739\n\
    - seed: 1008004056\n\
    - seed: 2015958364\n\
random_test2:\n\
  test_name: random_test\n\
  map_name: kashiwanoha_map\n\
  ego_goal_s: 0\n\
  ego_goal_lanelet_id: -1\n\
  ego_goal_partial_randomization: false\n\
  ego_goal_partial_randomization_distance: 25\n\
  npc_count: 10\n\
  npc_min_speed: 0.5\n\
  npc_max_speed: 3\n\
  npc_min_spawn_distance_from_ego: 10\n\
  npc_max_spawn_distance_from_ego: 100\n\
  test_cases:\n\
    - seed: 121660883\n\
    - seed: 478515236\n\
    - seed: 3404285739\n\
    - seed: 1008004056\n\
    - seed: 2015958364";
  writeToFile(file_data);

  EXPECT_THROW(test_params_IO.read(), std::runtime_error);
}

TEST(YamlTestParamsSaver, YamlTestParamsIO_writeSingle)
{
  auto test_params_IO = YamlTestParamsIO(rclcpp::get_logger("yaml_test_params_saver"), "/tmp");
  TestSuiteParameters suite_parameters;
  TestCaseParameters test_case_parameters1;
  TestCaseParameters test_case_parameters2;
  test_case_parameters1.seed = 100;
  test_case_parameters2.seed = 200;
  test_params_IO.addTestSuite(suite_parameters, "Suite1");
  test_params_IO.addTestCase(test_case_parameters1, "Suite1");
  test_params_IO.addTestCase(test_case_parameters2, "Suite1");
  test_params_IO.write();

  std::string expected_file_data =
    "Suite1:\n\
  test_name: random_test\n\
  map_name: kashiwanoha_map\n\
  ego_goal_s: 0\n\
  ego_goal_lanelet_id: -1\n\
  ego_goal_partial_randomization: false\n\
  ego_goal_partial_randomization_distance: 30\n\
  npc_count: 10\n\
  npc_min_speed: 0.5\n\
  npc_max_speed: 3\n\
  npc_min_spawn_distance_from_ego: 10\n\
  npc_max_spawn_distance_from_ego: 100\n\
  test_cases:\n\
    - seed: 100\n\
    - seed: 200";

  EXPECT_STREQ(readFile().c_str(), expected_file_data.c_str());
}

TEST(YamlTestParamsSaver, YamlTestParamsIO_writeMultiple)
{
  auto test_params_IO = YamlTestParamsIO(rclcpp::get_logger("yaml_test_params_saver"), "/tmp");
  TestSuiteParameters suite_parameters1;
  TestSuiteParameters suite_parameters2;

  suite_parameters1.name = "random_test1";
  suite_parameters2.name = "random_test2";

  TestCaseParameters test_case_parameters1;
  TestCaseParameters test_case_parameters2;
  test_case_parameters1.seed = 100;
  test_case_parameters2.seed = 200;
  test_params_IO.addTestSuite(suite_parameters1, "Suite1");
  test_params_IO.addTestSuite(suite_parameters2, "Suite2");

  test_params_IO.addTestCase(test_case_parameters1, "Suite1");
  test_params_IO.addTestCase(test_case_parameters2, "Suite2");

  test_params_IO.write();

  std::string expected_file_data =
    "Suite1:\n\
  test_name: random_test1\n\
  map_name: kashiwanoha_map\n\
  ego_goal_s: 0\n\
  ego_goal_lanelet_id: -1\n\
  ego_goal_partial_randomization: false\n\
  ego_goal_partial_randomization_distance: 30\n\
  npc_count: 10\n\
  npc_min_speed: 0.5\n\
  npc_max_speed: 3\n\
  npc_min_spawn_distance_from_ego: 10\n\
  npc_max_spawn_distance_from_ego: 100\n\
  test_cases:\n\
    - seed: 100\n\
Suite2:\n\
  test_name: random_test2\n\
  map_name: kashiwanoha_map\n\
  ego_goal_s: 0\n\
  ego_goal_lanelet_id: -1\n\
  ego_goal_partial_randomization: false\n\
  ego_goal_partial_randomization_distance: 30\n\
  npc_count: 10\n\
  npc_min_speed: 0.5\n\
  npc_max_speed: 3\n\
  npc_min_spawn_distance_from_ego: 10\n\
  npc_max_spawn_distance_from_ego: 100\n\
  test_cases:\n\
    - seed: 200";

  EXPECT_STREQ(readFile().c_str(), expected_file_data.c_str());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
