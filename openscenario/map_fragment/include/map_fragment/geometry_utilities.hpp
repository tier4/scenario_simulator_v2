// Copyright 2015 TIER IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MAP_FRAGMENT__GEOMETRY_UTILITIES__HPP_
#define MAP_FRAGMENT__GEOMETRY_UTILITIEs__HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>
#include <string>

namespace map_fragment
{

using Point = Eigen::Vector3d;
using Vector = Eigen::Vector3d;

using Transformation = Eigen::Transform<double, 3, Eigen::TransformTraits::AffineCompact>;

auto rotateInZAxis(const Eigen::Vector3d & point_or_vector, const double angle) -> Eigen::Vector3d
{
  const auto rotation = Eigen::AngleAxis(angle, Eigen::Vector3d::UnitZ());
  const auto transformation = Transformation(rotation);
  return transformation * point_or_vector;
}

auto vectorToTranslation(const Vector & vector) -> Transformation
{
  const auto translation = Eigen::Translation<Vector::Scalar, 3>(vector);
  return Transformation(translation);
}

auto vectorToRotationWithZeroRoll(const Vector & vector) -> Transformation
{
  const auto magnitude = vector.norm();

  if (magnitude == 0) {
    throw std::invalid_argument("The input vector needs to have non-zero magnitude.");
  }

  if (vector.x() == 0 && vector.y() == 0) {
    throw std::invalid_argument(
      "The input vector must not be parallel to the z axis. "
      "Expected x or y coordinate to be non-zero.");
  }

  const auto yaw = std::atan2(vector.y(), vector.x());
  const auto pitch = std::asin(vector.z() / magnitude);

  const auto rotation =
    Eigen::AngleAxis(yaw, Vector::UnitZ()) * Eigen::AngleAxis(pitch, Vector::UnitY());
  return Transformation(rotation);
}

auto rotateInLocalZAxisAssumingZeroRoll(const Vector & vector, const double angle)
  -> Eigen::Vector3d
{
  const auto local_reference_frame_rotation = vectorToRotationWithZeroRoll(vector);
  const auto rotation_axis = local_reference_frame_rotation * Vector::UnitZ();
  const auto rotation = Eigen::AngleAxis(angle, rotation_axis);
  const auto transformation = Transformation(rotation);
  return transformation * vector;
}

auto chainTransformations(const Transformation & first, const Transformation & second)
  -> Transformation
{
  return second * first;
}

auto applyTransformationToPoint(const Point & point, const Transformation & transformation) -> Point
{
  return transformation * point;
}

auto applyTransformationToVector(const Vector & vector, const Transformation & transformation)
  -> Vector
{
  return transformation.rotation() * vector;
}

}  // namespace map_fragment

#endif  // MAP_FRAGMENT__GEOMETRY_UTILITIES__HPP_
