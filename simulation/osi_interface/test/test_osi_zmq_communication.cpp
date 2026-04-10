// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#include <chrono>
#include <osi_interface/osi_zmq_client.hpp>
#include <osi_interface/osi_zmq_server.hpp>
#include <thread>

using namespace osi_interface;

constexpr int kTestPort = 15556;
constexpr double kTol = 1e-9;

TEST(OsiZmqCommunication, ClientServerRoundTrip)
{
  // Server: echo back a TrafficUpdate with the same timestamp
  auto handler = [](const osi3::GroundTruth & gt) -> osi3::TrafficUpdate {
    osi3::TrafficUpdate tu;
    tu.mutable_version()->set_version_major(3);
    tu.mutable_version()->set_version_minor(8);
    tu.mutable_version()->set_version_patch(0);
    if (gt.has_timestamp()) {
      *tu.mutable_timestamp() = gt.timestamp();
    }
    // Echo back an object with the same position as the first moving object
    if (gt.moving_object_size() > 0) {
      *tu.add_update() = gt.moving_object(0);
    }
    return tu;
  };

  OsiZmqServer server(kTestPort, handler);
  server.start();

  // Give server time to bind
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  OsiZmqClient client("localhost", kTestPort);

  // Build a test GroundTruth
  osi3::GroundTruth gt;
  gt.mutable_timestamp()->set_seconds(42);
  gt.mutable_timestamp()->set_nanos(500000000);
  auto * obj = gt.add_moving_object();
  obj->mutable_id()->set_value(1);
  obj->mutable_base()->mutable_position()->set_x(100.0);
  obj->mutable_base()->mutable_position()->set_y(200.0);

  // Send and receive
  auto tu = client.sendGroundTruth(gt);

  // Verify
  EXPECT_EQ(tu.timestamp().seconds(), 42);
  EXPECT_EQ(tu.timestamp().nanos(), 500000000u);
  ASSERT_EQ(tu.update_size(), 1);
  EXPECT_NEAR(tu.update(0).base().position().x(), 100.0, kTol);
  EXPECT_NEAR(tu.update(0).base().position().y(), 200.0, kTol);

  client.close();
  server.stop();
}

TEST(OsiZmqCommunication, MultipleFrames)
{
  int frame_count = 0;
  auto handler = [&frame_count](const osi3::GroundTruth & gt) -> osi3::TrafficUpdate {
    frame_count++;
    osi3::TrafficUpdate tu;
    if (gt.has_timestamp()) {
      *tu.mutable_timestamp() = gt.timestamp();
    }
    return tu;
  };

  OsiZmqServer server(kTestPort + 1, handler);
  server.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  OsiZmqClient client("localhost", kTestPort + 1);

  for (int i = 0; i < 5; ++i) {
    osi3::GroundTruth gt;
    gt.mutable_timestamp()->set_seconds(i);
    auto tu = client.sendGroundTruth(gt);
    EXPECT_EQ(tu.timestamp().seconds(), i);
  }

  EXPECT_EQ(frame_count, 5);

  client.close();
  server.stop();
}
