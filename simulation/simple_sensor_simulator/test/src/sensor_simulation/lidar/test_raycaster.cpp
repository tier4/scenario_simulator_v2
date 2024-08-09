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

#include "test_raycaster.hpp"

/**
 * @note Test basic functionality. Test adding a primitive to scene correctness with a Box.
 */
TEST_F(RaycasterTest, addPrimitive_box)
{
  EXPECT_NO_THROW(raycaster_->addPrimitive<primitives::Box>(
    box_name_, box_depth_, box_width_, box_height_, box_pose_));
}

/**
 * @note Test function behavior when adding two Boxes with identical names to a scene.
 */
TEST_F(RaycasterTest, addPrimitive_twoIdenticalNames)
{
  raycaster_->addPrimitive<primitives::Box>(
    box_name_, box_depth_, box_width_, box_height_, box_pose_);

  EXPECT_THROW(
    raycaster_->addPrimitive<primitives::Box>(
      box_name_, box_depth_, box_width_, box_height_, box_pose_),
    std::runtime_error);
}

/**
 * @note Test basic functionality. Test raycasting correctness with an empty scene.
 */
TEST_F(RaycasterTest, raycast_empty)
{
  const auto cloud = raycaster_->raycast(frame_id_, stamp_, origin_);
  const auto total_num_of_points = cloud.width * cloud.height;

  EXPECT_EQ(total_num_of_points, 0);
  EXPECT_EQ(cloud.header.frame_id, frame_id_);
  EXPECT_EQ(cloud.header.stamp, stamp_);
}

/**
 * @note Test basic functionality. Test raycasting correctness with one box on the scene.
 */
TEST_F(RaycasterTest, raycast_box)
{
  raycaster_->addPrimitive<primitives::Box>(
    box_name_, box_depth_, box_width_, box_height_, box_pose_);

  const auto cloud = raycaster_->raycast(frame_id_, stamp_, origin_);
  const auto total_num_of_points = cloud.width * cloud.height;

  EXPECT_GT(total_num_of_points, 0);
  EXPECT_EQ(cloud.header.frame_id, frame_id_);
  EXPECT_EQ(cloud.header.stamp, stamp_);
}

/**
 * @note Test basic functionality. Test setting ray directions with a lidar configuration that has
 * one ray which intersects with the only box on the scene.
 */
TEST_F(RaycasterTest, setDirection_oneBox)
{
  raycaster_->addPrimitive<primitives::Box>(
    box_name_, box_depth_, box_width_, box_height_, box_pose_);

  simulation_api_schema::LidarConfiguration config;
  config.add_vertical_angles(0.0);  // Only one vertical angle for a horizontal ring
  config.set_horizontal_resolution(utils::degToRad(1.0));  // Set horizontal resolution to 1 degree

  raycaster_->setDirection(config);

  const auto cloud = raycaster_->raycast(frame_id_, stamp_, origin_);
  const auto total_num_of_points = cloud.width * cloud.height;

  EXPECT_GT(total_num_of_points, 0);
  EXPECT_EQ(cloud.header.frame_id, frame_id_);
  EXPECT_EQ(cloud.header.stamp, stamp_);
}

/**
 * @note Test basic functionality. Test setting ray directions with a lidar configuration that has a
 * ring of horizontal rays which intersect with boxes positioned on the ring so that they intersect
 * with the rays.
 */
TEST_F(RaycasterTest, setDirection_manyBoxes)
{
  constexpr double radius = 5.0;
  constexpr int num_boxes = 10;
  constexpr double angle_increment = 2.0 * M_PI / num_boxes;

  for (int i = 0; i < num_boxes; ++i) {
    const double angle = i * angle_increment;
    const auto box_pose =
      utils::makePose(radius * cos(angle), radius * sin(angle), 0.0, 0.0, 0.0, 0.0, 1.0);

    const std::string name = "box" + std::to_string(i);
    raycaster_->addPrimitive<primitives::Box>(name, box_depth_, box_width_, box_height_, box_pose);
  }

  simulation_api_schema::LidarConfiguration config;
  config.add_vertical_angles(0.0);  // Only one vertical angle for a horizontal ring
  config.set_horizontal_resolution(utils::degToRad(5.0));

  raycaster_->setDirection(config);

  const auto cloud = raycaster_->raycast(frame_id_, stamp_, origin_);
  const auto total_num_of_points = cloud.width * cloud.height;

  EXPECT_GT(total_num_of_points, 0);
  EXPECT_EQ(cloud.header.frame_id, frame_id_);
  EXPECT_EQ(cloud.header.stamp, stamp_);
}

/**
 * @note Test basic functionality. Test detected objects obtaining from the statuses list containing
 * Ego.
 */
TEST_F(RaycasterTest, getDetectedObjects)
{
  raycaster_->addPrimitive<primitives::Box>(
    box_name_, box_depth_, box_width_, box_height_, box_pose_);

  raycaster_->raycast(frame_id_, stamp_, origin_);

  const auto & detected_objects = raycaster_->getDetectedObject();

  ASSERT_FALSE(detected_objects.empty());
  EXPECT_EQ(detected_objects[0], box_name_);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
