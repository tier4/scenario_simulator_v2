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

struct DummyCanonicalizedLaneletPose
{
};

class MockTrafficSimulatorAPI
{
public:
  traffic_simulator::EntityStatus entity_status_;
  std::shared_ptr<::testing::StrictMock<MockFieldOperatorApplication>>
    field_operator_application_mock =
      std::make_shared<::testing::StrictMock<MockFieldOperatorApplication>>();

  MOCK_METHOD(bool, updateFrame, (), ());
  MOCK_METHOD(
    bool, spawn,
    (const std::string &, DummyCanonicalizedLaneletPose,
     const traffic_simulator_msgs::msg::VehicleParameters &, const std::string &),
    ());
  MOCK_METHOD(
    void, setEntityStatus,
    (const std::string &, DummyCanonicalizedLaneletPose,
     const traffic_simulator_msgs::msg::ActionStatus),
    ());
  MOCK_METHOD(void, attachLidarSensor, (const simulation_api_schema::LidarConfiguration &), ());
  MOCK_METHOD(
    void, attachDetectionSensor, (const simulation_api_schema::DetectionSensorConfiguration &), ());
  MOCK_METHOD(
    bool, attachOccupancyGridSensor, (simulation_api_schema::OccupancyGridSensorConfiguration), ());
  MOCK_METHOD(void, asFieldOperatorApplicationMock, (const std::string &), ());
  MOCK_METHOD(
    void, requestAssignRoute, (const std::string &, std::vector<DummyCanonicalizedLaneletPose>),
    ());
  MOCK_METHOD(
    DummyCanonicalizedLaneletPose, canonicalize, (const traffic_simulator::LaneletPose &), ());
  MOCK_METHOD(
    void, spawn,
    (const std::string &, DummyCanonicalizedLaneletPose,
     const traffic_simulator_msgs::msg::VehicleParameters &),
    ());
  MOCK_METHOD(void, requestSpeedChange, (const std::string &, double, bool), ());
  MOCK_METHOD(bool, isEgoSpawned, (), ());
  MOCK_METHOD(bool, isNpcLogicStarted, (), ());
  MOCK_METHOD(void, startNpcLogic, (), ());
  MOCK_METHOD(bool, despawn, (const std::string), ());
  MOCK_METHOD(std::string, getEgoName, (), ());
  MOCK_METHOD(double, getCurrentTime, (), ());
  MOCK_METHOD(void, getEntityStatusMock, (const std::string &), ());
  MOCK_METHOD(bool, entityExists, (const std::string &), ());
  MOCK_METHOD(bool, checkCollision, (const std::string &, const std::string &), ());

  ::testing::StrictMock<MockFieldOperatorApplication> & asFieldOperatorApplication(
    const std::string & name)
  {
    asFieldOperatorApplicationMock(name);
    return *field_operator_application_mock;
  }

  traffic_simulator::CanonicalizedEntityStatus getEntityStatus(const std::string & name)
  {
    getEntityStatusMock(name);
    entity_status_.lanelet_pose_valid = false;
    std::shared_ptr<hdmap_utils::HdMapUtils> hd_map_utils_nullptr = nullptr;
    return traffic_simulator::CanonicalizedEntityStatus(entity_status_, hd_map_utils_nullptr);
  }

  void setEntityStatusNecessaryValues(
    double time, geometry_msgs::msg::Pose map_pose, geometry_msgs::msg::Twist twist)
  {
    entity_status_.time = time;
    entity_status_.pose = map_pose;
    entity_status_.action_status.twist = twist;
  }
};

TEST(TestExecutor, InitializeWithNoNPCs)
{
  ::testing::Sequence sequence;
  auto MockAPI = std::make_shared<::testing::StrictMock<MockTrafficSimulatorAPI>>();

  auto test_case = common::junit::SimpleTestCase("test_case");
  auto test_executor = TestExecutor<MockTrafficSimulatorAPI>(
    MockAPI, TestDescription(), JunitXmlReporterTestCase(test_case),
    SimulatorType::SIMPLE_SENSOR_SIMULATOR, ArchitectureType::AWF_UNIVERSE,
    rclcpp::get_logger("test_executor_test"));

  EXPECT_CALL(*MockAPI, updateFrame).Times(1).InSequence(sequence);
  EXPECT_CALL(*MockAPI, canonicalize).Times(1).InSequence(sequence);
  EXPECT_CALL(
    *MockAPI, spawn(
                ::testing::A<const std::string &>(), ::testing::A<DummyCanonicalizedLaneletPose>(),
                ::testing::A<const traffic_simulator_msgs::msg::VehicleParameters &>(),
                ::testing::A<const std::string &>()))
    .Times(1)
    .InSequence(sequence);
  EXPECT_CALL(*MockAPI, canonicalize).Times(1).InSequence(sequence);
  EXPECT_CALL(*MockAPI, setEntityStatus).Times(1).InSequence(sequence);
  EXPECT_CALL(*MockAPI, attachLidarSensor).Times(1).InSequence(sequence);
  EXPECT_CALL(*MockAPI, attachDetectionSensor).Times(1).InSequence(sequence);
  EXPECT_CALL(*MockAPI, attachOccupancyGridSensor).Times(1).InSequence(sequence);
  EXPECT_CALL(*MockAPI, asFieldOperatorApplicationMock).Times(1).InSequence(sequence);
  EXPECT_CALL(*(MockAPI->field_operator_application_mock), declare_parameter_mock)
    .Times(1)
    .InSequence(sequence);
  EXPECT_CALL(*MockAPI, canonicalize).Times(1).InSequence(sequence);
  EXPECT_CALL(*MockAPI, requestAssignRoute).Times(1).InSequence(sequence);
  EXPECT_CALL(*MockAPI, asFieldOperatorApplicationMock).Times(1).InSequence(sequence);
  EXPECT_CALL(*(MockAPI->field_operator_application_mock), engage).Times(1).InSequence(sequence);

  test_executor.initialize();
}

TEST(TestExecutor, UpdateNoNPCs)
{
  ::testing::Sequence sequence;
  auto MockAPI = std::make_shared<::testing::StrictMock<MockTrafficSimulatorAPI>>();

  auto test_case = common::junit::SimpleTestCase("test_case");
  auto test_executor = TestExecutor<MockTrafficSimulatorAPI>(
    MockAPI, TestDescription(), JunitXmlReporterTestCase(test_case),
    SimulatorType::SIMPLE_SENSOR_SIMULATOR, ArchitectureType::AWF_UNIVERSE,
    rclcpp::get_logger("test_executor_test"));

  EXPECT_CALL(*MockAPI, isEgoSpawned)
    .Times(1)
    .InSequence(sequence)
    .WillOnce(::testing::Return(false));
  EXPECT_CALL(*MockAPI, isNpcLogicStarted)
    .Times(1)
    .InSequence(sequence)
    .WillOnce(::testing::Return(false));
  EXPECT_CALL(*MockAPI, startNpcLogic).Times(1).InSequence(sequence);
  EXPECT_CALL(*MockAPI, isEgoSpawned)
    .Times(1)
    .InSequence(sequence)
    .WillOnce(::testing::Return(false));
  EXPECT_CALL(*MockAPI, getCurrentTime)
    .Times(1)
    .InSequence(sequence)
    .WillOnce(::testing::Return(1.0));
  EXPECT_CALL(*MockAPI, getEntityStatusMock).Times(1).InSequence(sequence);
  EXPECT_CALL(*MockAPI, updateFrame).Times(1).InSequence(sequence);

  test_executor.update();
}

TEST(TestExecutor, DeinitializeWithNoNPCs)
{
  auto MockAPI = std::make_shared<::testing::StrictMock<MockTrafficSimulatorAPI>>();
  auto test_case = common::junit::SimpleTestCase("test_case");
  auto test_executor = TestExecutor<MockTrafficSimulatorAPI>(
    MockAPI, TestDescription(), JunitXmlReporterTestCase(test_case),
    SimulatorType::SIMPLE_SENSOR_SIMULATOR, ArchitectureType::AWF_UNIVERSE,
    rclcpp::get_logger("test_executor_test"));

  EXPECT_CALL(*MockAPI, despawn("ego")).Times(1);

  test_executor.deinitialize();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
