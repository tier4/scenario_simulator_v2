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

#include "test_lidar_sensor.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <cstdint>
#include <simple_sensor_simulator/sensor_simulation/lidar/object_compatibility.hpp>

/**
 * @note Test function behavior when called on a scene without Ego entity added - the goal is to
 * test error throwing.
 */
TEST_F(LidarSensorTest, update_noEgo)
{
  status_.clear();  // Remove ego
  EXPECT_THROW(
    lidar_->update(current_simulation_time_, status_, current_ros_time_), std::runtime_error);
}

/**
 * @note Test basic functionality. Test lidar sensor correctness on a sample scene with some vehicle
 * - the goal is to check if the correct pointcloud is published on the correct topic.
 */
TEST_F(LidarSensorTest, update_correct)
{
  lidar_->update(current_simulation_time_, status_, current_ros_time_);

  // Spin the node to process callbacks
  executor_->spin_some();

  ASSERT_TRUE(received_msg_);
  const auto total_num_of_points = received_msg_->width * received_msg_->height;
  EXPECT_GT(total_num_of_points, 0);
  EXPECT_EQ(received_msg_->header.frame_id, "base_link");
  // An architecture_type older than PointXYZCPE must keep the layout existing scenarios see.
  EXPECT_EQ(received_msg_->point_step, sizeof(pcl::PointXYZI));
}

/**
 * @note Test function behavior when called with a current_time significantly smaller than one call
 * earlier - the goal is to test whether the function clears detected_objects.
 */
TEST_F(LidarSensorTest, update_goBackInTime)
{
  lidar_->update(current_simulation_time_, status_, rclcpp::Time(1000));

  // Ensure there are detected objects
  ASSERT_FALSE(lidar_->getDetectedObjects().empty());

  lidar_->update(current_simulation_time_, status_, rclcpp::Time(1));

  // Spin the node to process callbacks
  executor_->spin_some();

  EXPECT_TRUE(lidar_->getDetectedObjects().empty());
}

/**
 * @note Test basic functionality. Test detected objects obtaining from the statuses list containing
 * Ego.
 */
TEST_F(LidarSensorTest, getDetectedObjects)
{
  const std::set<std::string> expected_objects = {
    status_[1].name(), status_[2].name(), status_[3].name()};

  lidar_->update(current_simulation_time_, status_, current_ros_time_);

  // Spin the node to process callbacks
  executor_->spin_some();

  const auto & detected_objects = lidar_->getDetectedObjects();

  // LidarSensor returns duplicates. To avoid them, a std::set is used.
  const std::set<std::string> unique_objects(detected_objects.begin(), detected_objects.end());

  ASSERT_FALSE(unique_objects.empty());
  EXPECT_EQ(unique_objects, expected_objects);
}

/**
 * @note Test the cloud published as PointXYZCPE. Downstream rejects a cloud whose fields or
 * point_step differ, and every point here comes from the misc object, whose classification is
 * STRUCTURE with the maximal confidence of an exact, simulated classification.
 */
TEST_F(SegmentedLidarSensorTest, segmentedPointCloud)
{
  lidar_->update(current_simulation_time_, status_, current_ros_time_);
  executor_->spin_some();

  ASSERT_TRUE(received_msg_);
  ASSERT_EQ(received_msg_->point_step, sizeof(PointXYZCPE));
  ASSERT_EQ(received_msg_->point_step, 24u);

  std::vector<std::string> field_names;
  for (const auto & field : received_msg_->fields) {
    field_names.push_back(field.name);
  }
  EXPECT_EQ(
    field_names, (std::vector<std::string>{"x", "y", "z", "class_id", "probability", "entropy"}));

  pcl::PointCloud<PointXYZCPE> cloud;
  pcl::fromROSMsg(*received_msg_, cloud);
  ASSERT_FALSE(cloud.empty()) << "the misc object of the fixture should have put points here";

  for (const auto & point : cloud) {
    EXPECT_EQ(point.class_id, static_cast<std::uint8_t>(PointCloudClassification::STRUCTURE));
    EXPECT_FLOAT_EQ(point.probability, 1.0f);
    EXPECT_FLOAT_EQ(point.entropy, 0.0f);
  }
}

/**
 * @note Test the mappings the published cloud cannot show. Only the misc object branch reaches the
 * wire, because every other classification is object compatible and those points are dropped.
 */
TEST(SegmentedPointCloud, classificationOf)
{
  const auto pose = utils::makePose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  const auto dimensions = utils::makeDimensions(4.5, 2.0, 1.5);

  EXPECT_EQ(
    classificationOf(
      utils::makeEntity("entity", EntityType::VEHICLE, EntitySubtype::TRAILER, pose, dimensions)),
    PointCloudClassification::TRUCK)
    << "PointCloudClassification has no TRAILER, and upstream try_into_pointcloud maps it to TRUCK";
  EXPECT_EQ(
    classificationOf(utils::makeEntity("entity", EntityType::VEHICLE, pose, dimensions)),
    PointCloudClassification::CAR)
    << "an entity without a subtype falls back to its type";
  EXPECT_EQ(
    classificationOf(utils::makeEntity("entity", EntityType::MISC_OBJECT, pose, dimensions)),
    PointCloudClassification::STRUCTURE)
    << "a misc object is the only way a scenario can place static world geometry";
}
