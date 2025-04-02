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

#include <gmock/gmock.h>

#include <random_test_runner/test_executor.hpp>
#include <traffic_simulator/api/configuration.hpp>
#include <traffic_simulator/utils/lanelet_map.hpp>

/// This class is for all entities to keep it simple
class MockEntity
{
public:
  MOCK_METHOD(
    void, setStatus,
    (const std::optional<traffic_simulator::CanonicalizedLaneletPose> &,
     const traffic_simulator_msgs::msg::ActionStatus &),
    ());
  MOCK_METHOD(void, requestSpeedChange, (double, bool), ());
  MOCK_METHOD(
    const traffic_simulator::CanonicalizedEntityStatus &, getCanonicalizedStatus, (), (const));

  // Ego member functions
  MOCK_METHOD(void, engage, (), ());
  MOCK_METHOD(bool, isEngageable, (), (const));
  MOCK_METHOD(bool, setParameterMock, (const std::string &, const bool &), (const));
  MOCK_METHOD(void, requestAssignRoute, (const std::vector<geometry_msgs::msg::Pose> &), ());

  template <typename ParameterType>
  auto setParameter(const std::string & name, const ParameterType & default_value = {}) const
    -> ParameterType
  {
    if constexpr (std::is_same_v<ParameterType, bool>) {
      setParameterMock(name, default_value);
      return default_value;
    }
    throw std::runtime_error("Unexpected typename in MockEntity::setParameter function");
  }
};

class MockFieldOperatorApplication
{
public:
  MOCK_METHOD(void, declare_parameter_mock, (), ());
  MOCK_METHOD(bool, engageable, (), ());
  MOCK_METHOD(void, engage, (), ());

  template <typename T>
  void declare_parameter(std::string, bool)
  {
    if constexpr (std::is_same_v<T, bool>) {
      declare_parameter_mock();
      return;
    }
    throw std::runtime_error("Unexpected typename");
  }
};

/// Simplest possible valid lanelet pose that can be converted to CanonicalizedLaneletPose
auto getTestDescription() -> TestDescription
{
  TestDescription td;
  td.ego_goal_position = traffic_simulator_msgs::build<traffic_simulator_msgs::msg::LaneletPose>()
                           .lanelet_id(34513)
                           .s(10.0)
                           .offset(0.0)
                           .rpy(geometry_msgs::msg::Vector3());

  td.ego_goal_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(
        geometry_msgs::build<geometry_msgs::msg::Point>().x(3704.31).y(73766.2).z(-0.875988))
      .orientation(
        geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.23587).w(
          0.971785));

  td.ego_start_position = traffic_simulator_msgs::build<traffic_simulator_msgs::msg::LaneletPose>()
                            .lanelet_id(34513)
                            .s(0.0)
                            .offset(0.0)
                            .rpy(geometry_msgs::msg::Vector3());

  return td;
}

class MockTrafficSimulatorAPI
{
public:
  MockTrafficSimulatorAPI()
  {
    const std::string path =
      ament_index_cpp::get_package_share_directory("random_test_runner") + "/map/lanelet2_map.osm";
    traffic_simulator::lanelet_map::activate(path);
  }

  traffic_simulator::EntityStatus entity_status_;
  std::shared_ptr<::testing::StrictMock<MockFieldOperatorApplication>>
    field_operator_application_mock =
      std::make_shared<::testing::StrictMock<MockFieldOperatorApplication>>();

  MOCK_METHOD(bool, updateFrame, (), ());
  MOCK_METHOD(
    bool, spawn,
    (const std::string &, const traffic_simulator::CanonicalizedLaneletPose &,
     const traffic_simulator_msgs::msg::VehicleParameters &, const std::string &, std::string),
    ());
  MOCK_METHOD(void, attachLidarSensor, (const simulation_api_schema::LidarConfiguration &), ());
  MOCK_METHOD(
    void, attachDetectionSensor, (const simulation_api_schema::DetectionSensorConfiguration &), ());
  MOCK_METHOD(
    bool, attachOccupancyGridSensor, (simulation_api_schema::OccupancyGridSensorConfiguration), ());
  MOCK_METHOD(
    void, requestAssignRoute, (const std::string &, std::vector<geometry_msgs::msg::Pose>), ());
  MOCK_METHOD(
    void, spawn,
    (const std::string &, const traffic_simulator::CanonicalizedLaneletPose &,
     const traffic_simulator_msgs::msg::VehicleParameters &),
    ());
  MOCK_METHOD(bool, isNpcLogicStarted, (), ());
  MOCK_METHOD(void, startNpcLogic, (), ());
  MOCK_METHOD(bool, despawn, (const std::string), ());
  MOCK_METHOD(std::string, getEgoName, (), ());
  MOCK_METHOD(double, getCurrentTime, (), ());
  MOCK_METHOD(void, getEntityMock, (const std::string &), (const));
  MOCK_METHOD(void, getEgoEntityMock, (const std::string &), (const));
  MOCK_METHOD(bool, isEntityExist, (const std::string &), ());
  MOCK_METHOD(bool, checkCollision, (const std::string &, const std::string &), ());

  auto getEntity(const std::string & name) -> ::testing::StrictMock<MockEntity> &
  {
    getEntityMock(name);
    if (name == ego_name_) {
      return *ego_entity_;
    } else {
      throw std::runtime_error("Entity " + name + " does not exist");
    }
  }

  auto getEgoEntity(const std::string & name) -> ::testing::StrictMock<MockEntity> &
  {
    getEgoEntityMock(name);
    return *ego_entity_;
  }

  auto setEntityStatusNecessaryValues(
    double time, geometry_msgs::msg::Pose map_pose, geometry_msgs::msg::Twist twist) -> void
  {
    entity_status_.time = time;
    entity_status_.pose = map_pose;
    entity_status_.action_status.twist = twist;
  }

  const std::string ego_name_ = "ego";
  const std::shared_ptr<::testing::StrictMock<MockEntity>> ego_entity_ =
    std::make_shared<::testing::StrictMock<MockEntity>>();
};

TEST(TestExecutor, InitializeWithNoNPCs)
{
  ::testing::Sequence sequence;
  auto MockAPI = std::make_shared<::testing::StrictMock<MockTrafficSimulatorAPI>>();
  auto & mock_ego = MockAPI->ego_entity_;

  auto test_case = common::junit::SimpleTestCase("test_case");
  auto test_executor = TestExecutor<MockTrafficSimulatorAPI>(
    MockAPI, getTestDescription(), JunitXmlReporterTestCase(test_case), 20.0,
    ArchitectureType::AWF_UNIVERSE, rclcpp::get_logger("test_executor_test"));

  EXPECT_CALL(*MockAPI, updateFrame).Times(1).InSequence(sequence);
  EXPECT_CALL(
    *MockAPI, spawn(
                ::testing::A<const std::string &>(),
                ::testing::A<const traffic_simulator::CanonicalizedLaneletPose &>(),
                ::testing::A<const traffic_simulator_msgs::msg::VehicleParameters &>(),
                ::testing::A<const std::string &>(), ::testing::A<std::string>()))
    .Times(1)
    .InSequence(sequence);
  EXPECT_CALL(*MockAPI, getEgoEntityMock).Times(1).InSequence(sequence);
  EXPECT_CALL(*mock_ego, setStatus).Times(1).InSequence(sequence);
  EXPECT_CALL(*MockAPI, attachLidarSensor).Times(1).InSequence(sequence);
  EXPECT_CALL(*MockAPI, attachDetectionSensor).Times(1).InSequence(sequence);
  EXPECT_CALL(*MockAPI, attachOccupancyGridSensor).Times(1).InSequence(sequence);
  EXPECT_CALL(*mock_ego, setParameterMock).Times(1).InSequence(sequence);
  EXPECT_CALL(*mock_ego, requestAssignRoute).Times(1).InSequence(sequence);
  EXPECT_CALL(*mock_ego, engage).Times(1).InSequence(sequence);

  test_executor.initialize();
}

TEST(TestExecutor, UpdateNoNPCs)
{
  ::testing::Sequence sequence;
  auto MockAPI = std::make_shared<::testing::StrictMock<MockTrafficSimulatorAPI>>();
  auto & mock_ego = MockAPI->ego_entity_;

  auto test_case = common::junit::SimpleTestCase("test_case");
  auto test_executor = TestExecutor<MockTrafficSimulatorAPI>(
    MockAPI, TestDescription(), JunitXmlReporterTestCase(test_case), 20.0,
    ArchitectureType::AWF_UNIVERSE, rclcpp::get_logger("test_executor_test"));
  traffic_simulator::CanonicalizedEntityStatus status(
    traffic_simulator_msgs::msg::EntityStatus(), std::nullopt);

  EXPECT_CALL(*MockAPI, isNpcLogicStarted)
    .Times(1)
    .InSequence(sequence)
    .WillOnce(::testing::Return(false));
  EXPECT_CALL(*MockAPI, isEntityExist)
    .Times(1)
    .InSequence(sequence)
    .WillOnce(::testing::Return(false));
  EXPECT_CALL(*MockAPI, startNpcLogic).Times(1).InSequence(sequence);
  EXPECT_CALL(*MockAPI, getCurrentTime)
    .Times(1)
    .InSequence(sequence)
    .WillOnce(::testing::Return(1.0));
  EXPECT_CALL(*MockAPI, getEntityMock).Times(1).InSequence(sequence);
  EXPECT_CALL(*mock_ego, getCanonicalizedStatus)
    .Times(1)
    .InSequence(sequence)
    .WillOnce(::testing::ReturnRef(status));
  EXPECT_CALL(*MockAPI, getEntityMock).Times(1).InSequence(sequence);
  EXPECT_CALL(*mock_ego, getCanonicalizedStatus)
    .Times(1)
    .InSequence(sequence)
    .WillOnce(::testing::ReturnRef(status));
  EXPECT_CALL(*MockAPI, updateFrame).Times(1).InSequence(sequence);

  test_executor.update();
}

TEST(TestExecutor, DeinitializeWithNoNPCs)
{
  auto MockAPI = std::make_shared<::testing::StrictMock<MockTrafficSimulatorAPI>>();
  auto test_case = common::junit::SimpleTestCase("test_case");
  auto test_executor = TestExecutor<MockTrafficSimulatorAPI>(
    MockAPI, TestDescription(), JunitXmlReporterTestCase(test_case), 20.0,
    ArchitectureType::AWF_UNIVERSE, rclcpp::get_logger("test_executor_test"));

  EXPECT_CALL(*MockAPI, despawn("ego")).Times(1);

  test_executor.deinitialize();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
