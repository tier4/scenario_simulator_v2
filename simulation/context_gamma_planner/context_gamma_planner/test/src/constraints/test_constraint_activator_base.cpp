// Copyright 2021 Tier IV, Inc All rights reserved.
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
#include <context_gamma_planner/constraints/constraint_activator_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traffic_simulator/data_type/lanelet_pose.hpp>
#include <traffic_simulator/helper/helper.hpp>

// class ConstraintActivatorTest : public ::testing::Test
// {
// protected:
//   static void SetUpTestCase() { rclcpp::init(0, nullptr); }
//   static void TearDownTestCase() { rclcpp::shutdown(); }
//   virtual void SetUp()
//   {
//     std::string path = ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map/lanelet2_map.osm";
//     geographic_msgs::msg::GeoPoint origin;
//     origin.latitude = 35.61836750154;
//     origin.longitude = 139.78066608243;
//     hdmap_utils_ptr = std::make_shared<hdmap_utils::HdMapUtils>(path, origin);
//     const auto test_info = ::testing::UnitTest::GetInstance()->current_test_info();
//     std::stringstream test_name;
//     test_name << test_info->test_case_name() << "_" << test_info->name();
//     node = std::make_shared<rclcpp::Node>("node", test_name.str());
//     activator = std::make_unique<context_gamma_planner::constraints::ConstraintActivatorBase>(
//       hdmap_utils_ptr, manager);
//   }
//   virtual void TearDown()
//   {
//     // activator.release();
//     node.reset();
//   }

// public:
//   std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr;
//   std::unique_ptr<context_gamma_planner::constraints::ConstraintActivatorBase> activator;
//   rclcpp::Node::SharedPtr node;
// };

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
