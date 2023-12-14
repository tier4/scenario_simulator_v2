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

#ifndef MAP_FRAGMENT__PARAMETRIC_CURVE__HPP_
#define MAP_FRAGMENT__PARAMETRIC_CURVE__HPP_

#include <map_fragment/geometric_operations.hpp>
#include <memory>
#include <rcpputils/asserts.hpp>
#include <utility>

namespace map_fragment
{

/**
 * Parametric representation of a smooth curve in 3D Cartesian space
 * 
 * By convention, the curve is assumed to always start at point (0, 0, 0)
 * and have a tangent vector there of [1, 0, 0]. Geometric trasformations
 * can be applied afterwards if other start positions and/or
 * directions are required.
 */
class ParametricCurve
{
public:
  using ConstSharedPointer = std::shared_ptr<const ParametricCurve>;

  /**
   * Calculate curve point for given parameter tangent ∈ [0, 1]
   */
  auto getPosition(const double tangent) const -> Point
  {
    validateCurveParameterOrThrow(tangent);
    return getPositionWithParameterValidated(tangent);
  }

  /**
   * Calculate curve unit tangent vector for given parameter tangent ∈ [0, 1]
   */
  auto getUnitTangentVector(const double tangent) const -> Vector
  {
    validateCurveParameterOrThrow(tangent);

    const auto tangent_vector = getUnitTangentVectorWithParameterValidated(tangent);

    rcpputils::assert_true(
      std::abs(tangent_vector.norm() - 1.0) < 1e-9, "The tangent vector must be unit");

    return tangent_vector;
  }

private:
  /**
   * Calculate curve point for given range-validated parameter tangent ∈ [0, 1]
   * 
   * **All derived classes need to implement this method.**
   */
  virtual auto getPositionWithParameterValidated(const double tangent) const -> Point = 0;

  /**
   * Calculate curve unit tangent vector for given range-validated parameter tangent ∈ [0, 1]
   * 
   * **All derived classes need to implement this method.**
   */
  virtual auto getUnitTangentVectorWithParameterValidated(const double tangent) const -> Vector = 0;

protected:
  auto validateCurveParameterOrThrow(const double tangent) const -> void
  {
    rcpputils::require_true(
      0 <= tangent && tangent <= 1,
      "Expected tangent to be in range [0, 1]. Actual value: " + std::to_string(tangent));
  }
};  // class ParametricCurve

/**
 * Straight line of given length
 */
class Straight : public ParametricCurve
{
public:
  const double length;

  explicit Straight(const double length) : length(length)
  {
    rcpputils::require_true(
      length > 0.0, "Expected length to be positive. Actual value: " + std::to_string(length));
  }

private:
  auto getPositionWithParameterValidated(const double tangent) const -> Point override
  {
    return {tangent * length, 0, 0};
  }

  auto getUnitTangentVectorWithParameterValidated(const double) const -> Vector override
  {
    return {1, 0, 0};
  }
};  // class Straight

/**
 * Arc of given radius and angle (positive to the left)
 */
class Arc : public ParametricCurve
{
  const Point center_;

public:
  const double radius;
  const double angle;

  explicit Arc(const double radius, const double angle)
  : center_(0, angle > 0 ? radius : -radius, 0), radius(radius), angle(angle)
  {
    rcpputils::require_true(
      radius > 0.0, "Expected radius to be positive. Actual value: " + std::to_string(radius));

    rcpputils::require_true(
      0 < std::abs(angle) && std::abs(angle) < 2 * M_PI,
      "Expected angle to be in range (0, 2 * PI) or (-2 * PI, 0). Actual value: " +
        std::to_string(angle));
  }

private:
  auto getPositionWithParameterValidated(const double tangent) const -> Point override
  {
    const auto theta = tangent * angle;
    const Point origin = {0, 0, 0};
    return center_ + rotateInZAxis(origin - center_, theta);
  }

  auto getUnitTangentVectorWithParameterValidated(const double tangent) const -> Vector override
  {
    const auto theta = tangent * angle;
    const Vector v = {1, 0, 0};
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
  const std::vector<ParametricCurve::ConstSharedPointer> curves;

  explicit CombinedCurve(const std::vector<ParametricCurve::ConstSharedPointer> & curves)
  : curves(curves)
  {
    rcpputils::require_true(
      curves.size() >= 2, "Number of curves to be combined must be two or more. Actual number: " +
                            std::to_string(curves.size()));

    auto current_transformation = Transformation::Identity();

    for (const auto & curve : curves) {
      transformations_.push_back(current_transformation);

      const auto origin = Point::Zero();
      const auto translation = vectorToTranslation(curve->getPosition(1.0) - origin);
      const auto rotation = vectorToRotationWithZeroRoll(curve->getUnitTangentVector(1.0));
      const auto local_transformation = chainTransformations(rotation, translation);

      current_transformation = chainTransformations(local_transformation, current_transformation);
    }
  }

private:
  auto getPositionWithParameterValidated(const double tangent) const -> Point override
  {
    const auto [curve_index, curve_tangent] = getCurveIndexAndParameter(tangent);
    const auto curve = curves[curve_index];
    const auto transformation = transformations_[curve_index];

    return applyTransformationToPoint(curve->getPosition(curve_tangent), transformation);
  }

  auto getUnitTangentVectorWithParameterValidated(const double tangent) const -> Vector override
  {
    const auto [curve_index, curve_tangent] = getCurveIndexAndParameter(tangent);
    const auto curve = curves[curve_index];
    const auto transformation = transformations_[curve_index];

    return applyTransformationToVector(curve->getUnitTangentVector(curve_tangent), transformation);
  }

  auto getCurveIndexAndParameter(const double tangent) const -> std::pair<std::size_t, double>
  {
    validateCurveParameterOrThrow(tangent);

    const auto range_width_per_curve = 1. / curves.size();
    const auto curve_index =
      std::min(static_cast<std::size_t>(tangent / range_width_per_curve), curves.size() - 1);
    const auto curve_tangent = tangent / range_width_per_curve - curve_index;

    return std::make_pair(curve_index, curve_tangent);
  }
};  // class CombinedCurve
}  // namespace map_fragment

#endif  // MAP_FRAGMENT__PARAMETRIC_CURVE__HPP_
