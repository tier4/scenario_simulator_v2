# Copyright 2015 TIER IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Co-developed by TIER IV, Inc. and Robotec.AI sp. z o.o.

ament_add_gtest(test_almost_standstill_metric test_almost_standstill_metric.cpp)
ament_add_gtest(test_ego_collision_metric test_ego_collision_metric.cpp)
ament_add_gtest(test_goal_reached_metric test_goal_reached_metric.cpp)
ament_add_gtest(test_data_types test_data_types.cpp)
ament_add_gtest(test_lanelet_utils test_lanelet_utils.cpp)
ament_add_gtest(test_randomizers test_randomizers.cpp)
ament_add_gtest(test_test_randomizer test_test_randomizer.cpp)
ament_add_gtest(test_junit_xml_reporter test_junit_xml_reporter.cpp)
ament_add_gtest(test_yaml_test_params_saver test_yaml_test_params_saver.cpp)
ament_add_gmock(test_test_executor test_test_executor.cpp)


target_link_libraries(test_almost_standstill_metric ${PROJECT_NAME})
target_link_libraries(test_ego_collision_metric ${PROJECT_NAME})
target_link_libraries(test_goal_reached_metric ${PROJECT_NAME})
target_link_libraries(test_data_types ${PROJECT_NAME})
target_link_libraries(test_lanelet_utils ${PROJECT_NAME})
target_link_libraries(test_randomizers ${PROJECT_NAME})
target_link_libraries(test_test_randomizer ${PROJECT_NAME})
target_link_libraries(test_junit_xml_reporter ${PROJECT_NAME})
target_link_libraries(test_yaml_test_params_saver ${PROJECT_NAME})
target_link_libraries(test_test_executor ${PROJECT_NAME})
