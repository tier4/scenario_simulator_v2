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
#include <rcpputils/asserts.hpp>
#include <stdexcept>
#include <string>

namespace map_fragment
{

using Point = Eigen::Vector3d;
using Vector = Eigen::Vector3d;

using Transformation = Eigen::Transform<double, 3, Eigen::TransformTraits::AffineCompact>;

/**
 * Rotate a point or vector in z axis by a given angle.
 */
auto rotateInZAxis(const Eigen::Vector3d & point_or_vector, const double angle) -> Eigen::Vector3d
{
  const auto rotation = Eigen::AngleAxis(angle, Eigen::Vector3d::UnitZ());
  const auto transformation = Transformation(rotation);
  return transformation * point_or_vector;
}

/**
 * Convert a vector encoding displacement into a translating transformation.
 */
auto vectorToTranslation(const Vector & vector) -> Transformation
{
  const auto translation = Eigen::Translation<Vector::Scalar, 3>(vector);
  return Transformation(translation);
}

/**
 * Convert a vector encoding direction into a rotating transformation.
 * 
 * The resulting Transformation will represent the rotation between the x axis
 * and the direction the vector is pointing towards. Because it is not possible
 * to extract full 3D rotation from the vector alone, an assumption is made
 * that the roll component of rotation is equal to zero.
 * 
 * To extract vector direction, its magnitude needs to be non-zero.
 * In order to avoid ambiguity due to gimbal lock, the vector also
 * must not be verticlal.
 */
auto vectorToRotationWithZeroRoll(const Vector & vector) -> Transformation
{
  const auto magnitude = vector.norm();

  rcpputils::require_true(magnitude != 0, "The input vector needs to have non-zero magnitude.");

  rcpputils::require_true(
    vector.x() != 0 || vector.y() != 0,
    "The input vector must not be parallel to the z axis. "
    "Expected x or y coordinate to be non-zero.");

  const auto yaw = std::atan2(vector.y(), vector.x());
  const auto pitch = std::asin(vector.z() / magnitude);

  const auto rotation =
    Eigen::AngleAxis(yaw, Vector::UnitZ()) * Eigen::AngleAxis(pitch, Vector::UnitY());
  return Transformation(rotation);
}

/**
 * Rotate a vector in z axis (in local coordinate frame) by a given angle.
 * 
 * The local coordinate frame is defined by the vector's direction.
 * Its x axis is pointing in the same direction as the vector, and the y axis
 * is assumed to point leftwards, horizontally (with zero roll).
 * 
 * The vector needs to have non-zero magnitude and must not be vertical.
 * 
 * @sa vectorToRotationWithZeroRoll
 */
auto rotateInLocalZAxisAssumingZeroRoll(const Vector & vector, const double angle)
  -> Eigen::Vector3d
{
  const auto local_reference_frame_rotation = vectorToRotationWithZeroRoll(vector);
  const auto rotation_axis = local_reference_frame_rotation * Vector::UnitZ();
  const auto rotation = Eigen::AngleAxis(angle, rotation_axis);
  const auto transformation = Transformation(rotation);
  return transformation * vector;
}

/**
 * Chain two transformations together, to be applied according to the order of the arguments.
 */
auto chainTransformations(const Transformation & first, const Transformation & second)
  -> Transformation
{
  return second * first;
}

/**
 * Apply a transformation to a point.
 */
auto applyTransformationToPoint(const Point & point, const Transformation & transformation) -> Point
{
  return transformation * point;
}

/**
 * Apply a transformation to a vector.
 * 
 * Translation part of the transformation has no effect on a vector, so only rotation will be applied.
 */
auto applyTransformationToVector(const Vector & vector, const Transformation & transformation)
  -> Vector
{
  return transformation.rotation() * vector;
}

}  // namespace map_fragment

#endif  // MAP_FRAGMENT__GEOMETRY_UTILITIES__HPP_
