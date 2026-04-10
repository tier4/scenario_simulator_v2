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

#include <cmath>
#include <osi_interface/orientation_conversion.hpp>

namespace osi_interface
{
// Tait-Bryan z-y'-x'' (yaw-pitch-roll) decomposition
// Matches tf2::Matrix3x3::getRPY behavior
auto quaternionToEuler(const Quaternion & q) -> EulerAngles
{
  // roll (x-axis rotation)
  const double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
  const double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation) - clamp to avoid NaN from asin
  const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
  double pitch;
  if (std::abs(sinp) >= 1.0) {
    pitch = std::copysign(M_PI / 2.0, sinp);  // gimbal lock
  } else {
    pitch = std::asin(sinp);
  }

  // yaw (z-axis rotation)
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  const double yaw = std::atan2(siny_cosp, cosy_cosp);

  return {roll, pitch, yaw};
}

auto eulerToQuaternion(const EulerAngles & e) -> Quaternion
{
  const double cr = std::cos(e.roll * 0.5);
  const double sr = std::sin(e.roll * 0.5);
  const double cp = std::cos(e.pitch * 0.5);
  const double sp = std::sin(e.pitch * 0.5);
  const double cy = std::cos(e.yaw * 0.5);
  const double sy = std::sin(e.yaw * 0.5);

  return {
    sr * cp * cy - cr * sp * sy,  // x
    cr * sp * cy + sr * cp * sy,  // y
    cr * cp * sy - sr * sp * cy,  // z
    cr * cp * cy + sr * sp * sy   // w
  };
}

auto toOsiOrientation(double qx, double qy, double qz, double qw) -> osi3::Orientation3d
{
  const auto euler = quaternionToEuler({qx, qy, qz, qw});
  osi3::Orientation3d ori;
  ori.set_roll(euler.roll);
  ori.set_pitch(euler.pitch);
  ori.set_yaw(euler.yaw);
  return ori;
}

auto fromOsiOrientation(const osi3::Orientation3d & ori) -> Quaternion
{
  return eulerToQuaternion({ori.roll(), ori.pitch(), ori.yaw()});
}

}  // namespace osi_interface
