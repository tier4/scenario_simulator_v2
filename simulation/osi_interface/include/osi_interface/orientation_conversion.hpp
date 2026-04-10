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

#ifndef OSI_INTERFACE__ORIENTATION_CONVERSION_HPP_
#define OSI_INTERFACE__ORIENTATION_CONVERSION_HPP_

#include <osi3/osi_common.pb.h>

namespace osi_interface
{
struct Quaternion
{
  double x, y, z, w;
};

struct EulerAngles
{
  double roll, pitch, yaw;
};

// Quaternion (x,y,z,w) → OSI Orientation3d (roll,pitch,yaw)
// Uses Tait-Bryan z-y'-x'' convention (same as tf2 RPY)
auto quaternionToEuler(const Quaternion & q) -> EulerAngles;

// OSI Orientation3d (roll,pitch,yaw) → Quaternion (x,y,z,w)
auto eulerToQuaternion(const EulerAngles & e) -> Quaternion;

// Convenience: populate osi3::Orientation3d from quaternion components
auto toOsiOrientation(double qx, double qy, double qz, double qw) -> osi3::Orientation3d;

// Convenience: extract quaternion from osi3::Orientation3d
auto fromOsiOrientation(const osi3::Orientation3d & ori) -> Quaternion;

}  // namespace osi_interface

#endif  // OSI_INTERFACE__ORIENTATION_CONVERSION_HPP_
