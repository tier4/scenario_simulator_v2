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

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>
#include <simple_sensor_simulator/sensor_simulation/detection_sensor/detection_sensor.hpp>

/**
 * @note Test basic functionality. Test detected objects obtaining from the statuses list with one
 * status further away than detection range and one closer.
 */
TEST(DetectionSensorBaseTest, getDetectedObjects)
{
  // This test is expected to fail because DetectionSensorBase
  // do not implement the getDetectedObjects function.
  EXPECT_TRUE(false);
}

/**
 * @note Test basic functionality. Test sensor pose obtaining from the statuses list containing Ego.
 */
TEST(DetectionSensorBaseTest, getSensorPose_correct)
{
  // This test is expected to fail because DetectionSensorBase
  // do not implement the getSensorPose function.
  EXPECT_TRUE(false);
}

/**
 * @note Test basic functionality. Test sensor pose obtaining from the statuses list not containing
 * Ego.
 */
TEST(DetectionSensorBaseTest, getSensorPose_noEgo)
{
  // This test is expected to fail because DetectionSensorBase
  // do not implement the getSensorPose function.
  EXPECT_TRUE(false);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
