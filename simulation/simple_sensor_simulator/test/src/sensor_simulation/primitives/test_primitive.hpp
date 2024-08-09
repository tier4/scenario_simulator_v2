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

#ifndef SIMPLE_SENSOR_SIMULATOR__TEST__TEST_PRIMITIVE_HPP_
#define SIMPLE_SENSOR_SIMULATOR__TEST__TEST_PRIMITIVE_HPP_

#include <gtest/gtest.h>

#include <memory>
#include <simple_sensor_simulator/sensor_simulation/primitives/box.hpp>
#include <simple_sensor_simulator/sensor_simulation/primitives/primitive.hpp>

#include "../../utils/helper_functions.hpp"

using namespace simple_sensor_simulator::primitives;
using namespace simple_sensor_simulator;

class PrimitiveTest : public ::testing::Test
{
protected:
  PrimitiveTest()
  : pose_(utils::makePose(1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0)),
    primitive_(std::make_unique<Box>(2.0f, 2.0f, 2.0f, pose_))
  {
  }

  geometry_msgs::msg::Pose pose_;
  std::unique_ptr<Primitive> primitive_;
};

#endif  // SIMPLE_SENSOR_SIMULATOR__TEST__TEST_PRIMITIVE_HPP_
