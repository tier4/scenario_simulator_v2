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

#ifndef SIMPLE_SENSOR_SIMULATOR__TEST_TEST_GRID_TRAVERSAL_HPP_
#define SIMPLE_SENSOR_SIMULATOR__TEST_TEST_GRID_TRAVERSAL_HPP_

#include <gtest/gtest.h>

#include <simple_sensor_simulator/sensor_simulation/occupancy_grid/grid_traversal.hpp>

using namespace simple_sensor_simulator;

class GridTraversalTest : public ::testing::Test
{
protected:
  GridTraversalTest() : traversal_(std::make_unique<GridTraversal>(0.0, 0.0, 3.0, 3.0)) {}

  std::unique_ptr<GridTraversal> traversal_;
};

#endif  // SIMPLE_SENSOR_SIMULATOR__TEST_TEST_GRID_TRAVERSAL_HPP_
