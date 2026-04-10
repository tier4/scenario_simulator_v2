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

#include <cmath>
#include <osi_interface/orientation_conversion.hpp>

using osi_interface::EulerAngles;
using osi_interface::eulerToQuaternion;
using osi_interface::fromOsiOrientation;
using osi_interface::Quaternion;
using osi_interface::quaternionToEuler;
using osi_interface::toOsiOrientation;

constexpr double kTolerance = 1e-12;

TEST(OrientationConversion, IdentityQuaternionGivesZeroEuler)
{
  const auto euler = quaternionToEuler({0, 0, 0, 1});
  EXPECT_NEAR(euler.roll, 0.0, kTolerance);
  EXPECT_NEAR(euler.pitch, 0.0, kTolerance);
  EXPECT_NEAR(euler.yaw, 0.0, kTolerance);
}

TEST(OrientationConversion, Yaw90Degrees)
{
  // Quaternion for 90 degree yaw: (0, 0, sin(pi/4), cos(pi/4))
  const double s = std::sin(M_PI / 4.0);
  const double c = std::cos(M_PI / 4.0);
  const auto euler = quaternionToEuler({0, 0, s, c});
  EXPECT_NEAR(euler.roll, 0.0, kTolerance);
  EXPECT_NEAR(euler.pitch, 0.0, kTolerance);
  EXPECT_NEAR(euler.yaw, M_PI / 2.0, kTolerance);
}

TEST(OrientationConversion, Roll90Degrees)
{
  const double s = std::sin(M_PI / 4.0);
  const double c = std::cos(M_PI / 4.0);
  const auto euler = quaternionToEuler({s, 0, 0, c});
  EXPECT_NEAR(euler.roll, M_PI / 2.0, kTolerance);
  EXPECT_NEAR(euler.pitch, 0.0, kTolerance);
  EXPECT_NEAR(euler.yaw, 0.0, kTolerance);
}

TEST(OrientationConversion, Pitch90Degrees)
{
  const double s = std::sin(M_PI / 4.0);
  const double c = std::cos(M_PI / 4.0);
  const auto euler = quaternionToEuler({0, s, 0, c});
  EXPECT_NEAR(euler.roll, 0.0, kTolerance);
  EXPECT_NEAR(euler.pitch, M_PI / 2.0, kTolerance);
  EXPECT_NEAR(euler.yaw, 0.0, kTolerance);
}

TEST(OrientationConversion, RoundTripFromEuler)
{
  const EulerAngles original{0.3, 0.2, 1.5};
  const auto q = eulerToQuaternion(original);
  const auto result = quaternionToEuler(q);
  EXPECT_NEAR(result.roll, original.roll, kTolerance);
  EXPECT_NEAR(result.pitch, original.pitch, kTolerance);
  EXPECT_NEAR(result.yaw, original.yaw, kTolerance);
}

TEST(OrientationConversion, RoundTripFromQuaternion)
{
  // Arbitrary normalized quaternion
  const double norm = std::sqrt(0.1 * 0.1 + 0.2 * 0.2 + 0.3 * 0.3 + 0.9 * 0.9);
  const Quaternion original{0.1 / norm, 0.2 / norm, 0.3 / norm, 0.9 / norm};
  const auto euler = quaternionToEuler(original);
  const auto result = eulerToQuaternion(euler);

  // Quaternions q and -q represent the same rotation
  const double sign = (result.w * original.w >= 0) ? 1.0 : -1.0;
  EXPECT_NEAR(result.x * sign, original.x, kTolerance);
  EXPECT_NEAR(result.y * sign, original.y, kTolerance);
  EXPECT_NEAR(result.z * sign, original.z, kTolerance);
  EXPECT_NEAR(result.w * sign, original.w, kTolerance);
}

TEST(OrientationConversion, GimbalLockPitchPositive90)
{
  // pitch = +pi/2 (gimbal lock)
  const EulerAngles input{0.0, M_PI / 2.0, 0.0};
  const auto q = eulerToQuaternion(input);
  const auto result = quaternionToEuler(q);
  EXPECT_NEAR(result.pitch, M_PI / 2.0, 1e-10);
}

TEST(OrientationConversion, GimbalLockPitchNegative90)
{
  // pitch = -pi/2 (gimbal lock)
  const EulerAngles input{0.0, -M_PI / 2.0, 0.0};
  const auto q = eulerToQuaternion(input);
  const auto result = quaternionToEuler(q);
  EXPECT_NEAR(result.pitch, -M_PI / 2.0, 1e-10);
}

TEST(OrientationConversion, NegativeAngles)
{
  const EulerAngles original{-0.5, -0.3, -2.0};
  const auto q = eulerToQuaternion(original);
  const auto result = quaternionToEuler(q);
  EXPECT_NEAR(result.roll, original.roll, kTolerance);
  EXPECT_NEAR(result.pitch, original.pitch, kTolerance);
  EXPECT_NEAR(result.yaw, original.yaw, kTolerance);
}

TEST(OrientationConversion, OsiOrientationRoundTrip)
{
  const auto ori = toOsiOrientation(0.1, 0.2, 0.3, 0.9);
  EXPECT_TRUE(ori.has_roll());
  EXPECT_TRUE(ori.has_pitch());
  EXPECT_TRUE(ori.has_yaw());

  const auto q = fromOsiOrientation(ori);
  const auto ori2 = toOsiOrientation(q.x, q.y, q.z, q.w);
  EXPECT_NEAR(ori2.roll(), ori.roll(), kTolerance);
  EXPECT_NEAR(ori2.pitch(), ori.pitch(), kTolerance);
  EXPECT_NEAR(ori2.yaw(), ori.yaw(), kTolerance);
}

TEST(OrientationConversion, MultipleArbitraryAnglesRoundTrip)
{
  const std::vector<EulerAngles> test_cases = {
    {0.0, 0.0, 0.0},        {M_PI, 0.0, 0.0}, {0.0, 0.0, M_PI},
    {0.1, 0.2, 0.3},        {-1.0, 0.5, 2.5}, {0.0, M_PI / 4.0, M_PI / 3.0},
    {M_PI / 6.0, 0.0, 0.0},
  };

  for (const auto & original : test_cases) {
    const auto q = eulerToQuaternion(original);
    const auto result = quaternionToEuler(q);
    EXPECT_NEAR(result.roll, original.roll, kTolerance)
      << "Failed for roll=" << original.roll << " pitch=" << original.pitch
      << " yaw=" << original.yaw;
    EXPECT_NEAR(result.pitch, original.pitch, kTolerance)
      << "Failed for roll=" << original.roll << " pitch=" << original.pitch
      << " yaw=" << original.yaw;
    EXPECT_NEAR(result.yaw, original.yaw, kTolerance)
      << "Failed for roll=" << original.roll << " pitch=" << original.pitch
      << " yaw=" << original.yaw;
  }
}
