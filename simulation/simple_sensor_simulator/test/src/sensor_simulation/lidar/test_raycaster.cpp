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
 * @note Test basic functionality. Test creating a Box entity.
 */
TEST_F(RaycasterTest, createEntity_box)
{
  std::vector<Raycaster::Entity> entities;
  EXPECT_NO_THROW(entities.emplace_back(
    box_name_, std::make_unique<primitives::Box>(box_depth_, box_width_, box_height_, box_pose_)));
  EXPECT_EQ(entities.size(), 1);
  EXPECT_EQ(entities[0].name, box_name_);
}

/**
 * @note Test basic functionality. Test raycasting correctness with an empty scene.
 */
TEST_F(RaycasterTest, raycast_empty)
{
  std::vector<Raycaster::Entity> entities;
  const auto result = raycaster_->raycast(origin_, entities);

  EXPECT_EQ(result.cloud->points.size(), 0);
}

/**
 * @note Test basic functionality. Test raycasting correctness with one box on the scene.
 */
TEST_F(RaycasterTest, raycast_box)
{
  std::vector<Raycaster::Entity> entities;
  entities.emplace_back(
    box_name_, std::make_unique<primitives::Box>(box_depth_, box_width_, box_height_, box_pose_));
  const auto result = raycaster_->raycast(origin_, entities);

  EXPECT_GT(result.cloud->points.size(), 0);
}

/**
 * @note Test basic functionality. Test setting ray directions with a lidar configuration that has
 * one ray which intersects with the only box on the scene.
 */
TEST_F(RaycasterTest, setDirection_oneBox)
{
  std::vector<Raycaster::Entity> entities;
  entities.emplace_back(
    box_name_, std::make_unique<primitives::Box>(box_depth_, box_width_, box_height_, box_pose_));

  simulation_api_schema::LidarConfiguration config;
  config.add_vertical_angles(0.0);  // Only one vertical angle for a horizontal ring
  config.set_horizontal_resolution(utils::degToRad(1.0));  // Set horizontal resolution to 1 degree

  raycaster_->setDirection(config);

  const auto result = raycaster_->raycast(origin_, entities);

  EXPECT_GT(result.cloud->points.size(), 0);
}

/**
 * @note Test basic functionality. Test setting ray directions with a lidar configuration that has a
 * ring of horizontal rays which intersect with boxes positioned on the ring so that they intersect
 * with the rays.
 */
TEST_F(RaycasterTest, setDirection_manyBoxes)
{
  std::vector<Raycaster::Entity> entities;
  constexpr double radius = 5.0;
  constexpr int num_boxes = 10;
  constexpr double angle_increment = 2.0 * M_PI / num_boxes;

  for (int i = 0; i < num_boxes; ++i) {
    const double angle = i * angle_increment;
    const auto box_pose =
      utils::makePose(radius * cos(angle), radius * sin(angle), 0.0, 0.0, 0.0, 0.0, 1.0);

    const std::string name = "box" + std::to_string(i);
    entities.emplace_back(
      name, std::make_unique<primitives::Box>(box_depth_, box_width_, box_height_, box_pose));
  }

  simulation_api_schema::LidarConfiguration config;
  config.add_vertical_angles(0.0);  // Only one vertical angle for a horizontal ring
  config.set_horizontal_resolution(utils::degToRad(5.0));

  raycaster_->setDirection(config);

  const auto result = raycaster_->raycast(origin_, entities);

  EXPECT_GT(result.cloud->points.size(), 0);
}

/**
 * @note Test basic functionality. Test detected objects obtaining from the statuses list containing
 * Ego.
 */
TEST_F(RaycasterTest, detected_unique_entity_names)
{
  std::vector<Raycaster::Entity> entities;
  entities.emplace_back(
    box_name_, std::make_unique<primitives::Box>(box_depth_, box_width_, box_height_, box_pose_));

  auto result = raycaster_->raycast(origin_, entities);

  const auto detected_objects = result.getDetectedEntityNames(entities);

  ASSERT_FALSE(detected_objects.empty());
  EXPECT_EQ(detected_objects.count(box_name_), 1);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
