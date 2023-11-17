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

#ifndef MAP_FRAGMENT__GEOMETRY__HPP_
#define MAP_FRAGMENT__GEOMETRY__HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

namespace map_fragment
{

using Point = Eigen::Vector3d;
using Vector = Eigen::Vector3d;

constexpr auto COORDINATE_X = 0;
constexpr auto COORDINATE_Y = 1;
constexpr auto COORDINATE_Z = 2;

using Transformation = Eigen::Transform<double, 3, Eigen::TransformTraits::AffineCompact>;

auto rotateInZAxis(Eigen::Vector3d const & point_or_vector, double angle) -> Eigen::Vector3d
{
  const auto rotation = Eigen::AngleAxis(angle, Eigen::Vector3d::UnitZ());
  const auto transformation = Transformation(rotation);
  return transformation * point_or_vector;
}

auto vectorToTranslation(Vector const & vector) -> Transformation
{
  const auto translation = Eigen::Translation<Vector::Scalar, 3>(vector);
  return Transformation(translation);
}

auto vectorToRotationWithZeroRoll(Vector const & vector) -> Transformation
{
  const auto magnitude = vector.norm();

  if (magnitude == 0) {
    throw std::invalid_argument("The input vector needs to have non-zero magnitude.");
  }

  if (vector(COORDINATE_X) == 0 && vector(COORDINATE_Y) == 0) {
    throw std::invalid_argument(
      "The input vector must not be parallel to the z axis. "
      "Expected x or y coordinate to be non-zero.");
  }

  const auto yaw = std::atan2(vector(COORDINATE_Y), vector(COORDINATE_X));
  const auto pitch = std::asin(vector(COORDINATE_Z) / magnitude);

  const auto rotation =
    Eigen::AngleAxis(yaw, Vector::UnitZ()) * Eigen::AngleAxis(pitch, Vector::UnitY());
  return Transformation(rotation);
}

auto chainTransformations(Transformation const & first, Transformation const & second)
  -> Transformation
{
  return second * first;
}

auto applyTransformation(Point const & point, Transformation const & transformation) -> Point
{
  return transformation * point;
}

/**
 * Parametric representation of a smooth curve in 2D Cartesian space
 * 
 * By convention, the curve is assumed to always start at point (0, 0)
 * and have a tangent vector there of [1, 0]. Geometric trasformations
 * can be applied afterwards if other start positions and/or
 * directions are required.
 */
class ParametricCurve
{
public:
  using Pointer = std::shared_ptr<ParametricCurve>;

  /**
   * Calculate curve point for given parameter tangent ∈ [0, 1]
   */
  auto getPosition(double tangent) const -> Point
  {
    validateCurveParameterOrThrow(tangent);
    return getPosition_(tangent);
  }

  /**
   * Calculate curve tangent vector for given parameter tangent ∈ [0, 1]
   */
  auto getTangentVector(double tangent) const -> Vector
  {
    validateCurveParameterOrThrow(tangent);

    // TODO Throw error if the vector is not unit?
    return getTangentVector_(tangent);
  }

private:
  virtual auto getPosition_(double tangent) const -> Point = 0;
  virtual auto getTangentVector_(double tangent) const -> Vector = 0;

protected:
  auto validateCurveParameterOrThrow(double tangent) const -> void
  {
    if (!(0 <= tangent && tangent <= 1)) {
      throw std::invalid_argument(
        "Expected tangent to be in range [0, 1]. Actual value: " + std::to_string(tangent));
    }
  }
};  // class ParametricCurve

/**
 * Straight line of given length
 */
class Straight : public ParametricCurve
{
public:
  const double length;

  explicit Straight(double length) : length(length)
  {
    if (length <= 0.0) {
      throw std::invalid_argument(
        "Expected length to be positive. Actual value: " + std::to_string(length));
    }
  }

private:
  auto getPosition_(double tangent) const -> Point override { return {tangent * length, 0, 0}; }

  auto getTangentVector_(double) const -> Vector override { return {1, 0, 0}; }
};  // class Straight

/**
 * Arc of given radius and angle (positive to the left)
 */
class Arc : public ParametricCurve
{
  Point center_;

public:
  const double radius;
  const double angle;

  explicit Arc(double radius, double angle) : radius(radius), angle(angle)
  {
    if (radius <= 0.0) {
      throw std::invalid_argument(
        "Expected radius to be positive. Actual value:" + std::to_string(radius));
    }

    if (!(0 < std::abs(angle) && std::abs(angle) < 2 * M_PI)) {
      throw std::invalid_argument(
        "Expected angle to be in range (0, 2 * PI) or (-2 * PI, 0). Actual value: " +
        std::to_string(angle));
    }

    center_(COORDINATE_X) = 0;
    center_(COORDINATE_Y) = 0;
    center_(COORDINATE_Y) = angle > 0 ? radius : -radius;
  }

private:
  auto getPosition_(double tangent) const -> Point override
  {
    auto theta = tangent * angle;
    Point origin = {0, 0, 0};
    return center_ + rotateInZAxis(origin - center_, theta);
  }

  auto getTangentVector_(double tangent) const -> Vector override
  {
    auto theta = tangent * angle;
    Vector v = {1, 0, 0};
    return rotateInZAxis(v, theta);
  }
};  // class Arc

/**
 * Combination of two or more component curves, chained after one another
 */
class CombinedCurve : public ParametricCurve
{
  std::vector<Transformation> transformations_;

public:
  const std::vector<ParametricCurve::Pointer> curves;

  explicit CombinedCurve(std::vector<ParametricCurve::Pointer> const & curves) : curves(curves)
  {
    if (curves.size() < 2) {
      throw std::invalid_argument(
        "Number of curves to be combined must be two or more. Actual number: " +
        std::to_string(curves.size()));
    }

    Transformation current_transformation;

    for (const auto & curve : curves) {
      transformations_.push_back(current_transformation);

      const auto origin = Point::Zero();
      const auto translation = vectorToTranslation(curve->getPosition(1.0) - origin);
      const auto rotation = vectorToRotationWithZeroRoll(curve->getTangentVector(1.0));
      const auto local_transformation = chainTransformations(rotation, translation);

      current_transformation = chainTransformations(current_transformation, local_transformation);
    }
  }

private:
  auto getPosition_(double tangent) const -> Point override
  {
    auto [curve_id, curve_tangent] = getCurveIdAndParameter(tangent);
    auto curve = curves[curve_id];
    auto transformation = transformations_[curve_id];

    return applyTransformation(curve->getPosition(curve_tangent), transformation);
  }

  auto getTangentVector_(double tangent) const -> Vector override
  {
    auto [curve_id, curve_tangent] = getCurveIdAndParameter(tangent);
    auto curve = curves[curve_id];
    auto transformation = transformations_[curve_id];

    return applyTransformation(curve->getTangentVector(curve_tangent), transformation);
  }

  auto getCurveIdAndParameter(double tangent) const -> std::pair<std::size_t, double>
  {
    validateCurveParameterOrThrow(tangent);

    auto range_width_per_curve = 1. / curves.size();
    auto curve_id =
      std::min(static_cast<std::size_t>(tangent / range_width_per_curve), curves.size() - 1);
    auto curve_tangent = tangent / range_width_per_curve - curve_id;

    return std::make_pair(curve_id, curve_tangent);
  }
};  // class CombinedCurve

}  // namespace map_fragment

#endif  // MAP_FRAGMENT__GEOMETRY__HPP_
