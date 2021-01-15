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

#include <simulation_api/metrics/metrics_manager.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

class TestMetric : metrics::MetricBase
{
public:
  TestMetric()
  : metrics::MetricBase("ego", "test") {}
  void calculate()
  {
    foundSpecificationViolation("error");
  }
};

TEST(Metrics, AddMetrics)
{
  auto node = std::make_shared<rclcpp::Node>("metrics_test_node", "/metrics_test");
  std::string map_path = ament_index_cpp::get_package_share_directory(
    "kashiwanoha_map") + "/map/lanelet2_map.osm";
  simulation_api::entity::EntityManager manager(node, map_path);
  node.reset();
  // metrics::MetricsManager manager;
  // manager.addMetrics();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
