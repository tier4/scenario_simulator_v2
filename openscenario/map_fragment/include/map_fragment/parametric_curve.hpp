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

#include <map_fragment/geometry_utilities.hpp>
#include <memory>
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
  using ConstPointer = std::shared_ptr<const ParametricCurve>;

  /**
   * Calculate curve point for given parameter tangent ∈ [0, 1]
   */
  auto getPosition(const double tangent) const -> Point
  {
    validateCurveParameterOrThrow(tangent);
    return getPosition_(tangent);
  }

  /**
   * Calculate curve tangent vector for given parameter tangent ∈ [0, 1]
   */
  auto getTangentVector(const double tangent) const -> Vector
  {
    validateCurveParameterOrThrow(tangent);

    // TODO Throw error if the vector is not unit?
    return getTangentVector_(tangent);
  }

private:
  /**
   * Calculate curve point for given range-validated parameter tangent ∈ [0, 1]
   * 
   * **All derived classes need to implement this method.**
   */
  virtual auto getPosition_(const double tangent) const -> Point = 0;

  /**
   * Calculate curve tangent vector for given range-validated parameter tangent ∈ [0, 1]
   * 
   * **All derived classes need to implement this method.**
   */
  virtual auto getTangentVector_(const double tangent) const -> Vector = 0;

protected:
  auto validateCurveParameterOrThrow(const double tangent) const -> void
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

  explicit Straight(const double length) : length(length)
  {
    if (length <= 0.0) {
      throw std::invalid_argument(
        "Expected length to be positive. Actual value: " + std::to_string(length));
    }
  }

private:
  auto getPosition_(const double tangent) const -> Point override
  {
    return {tangent * length, 0, 0};
  }

  auto getTangentVector_(const double) const -> Vector override { return {1, 0, 0}; }
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
    if (radius <= 0.0) {
      throw std::invalid_argument(
        "Expected radius to be positive. Actual value:" + std::to_string(radius));
    }

    if (!(0 < std::abs(angle) && std::abs(angle) < 2 * M_PI)) {
      throw std::invalid_argument(
        "Expected angle to be in range (0, 2 * PI) or (-2 * PI, 0). Actual value: " +
        std::to_string(angle));
    }
  }

private:
  auto getPosition_(const double tangent) const -> Point override
  {
    const auto theta = tangent * angle;
    const Point origin = {0, 0, 0};
    return center_ + rotateInZAxis(origin - center_, theta);
  }

  auto getTangentVector_(const double tangent) const -> Vector override
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
  const std::vector<ParametricCurve::ConstPointer> curves;

  explicit CombinedCurve(const std::vector<ParametricCurve::ConstPointer> & curves) : curves(curves)
  {
    if (curves.size() < 2) {
      throw std::invalid_argument(
        "Number of curves to be combined must be two or more. Actual number: " +
        std::to_string(curves.size()));
    }

    auto current_transformation = Transformation::Identity();

    for (const auto & curve : curves) {
      transformations_.push_back(current_transformation);

      const auto origin = Point::Zero();
      const auto translation = vectorToTranslation(curve->getPosition(1.0) - origin);
      const auto rotation = vectorToRotationWithZeroRoll(curve->getTangentVector(1.0));
      const auto local_transformation = chainTransformations(rotation, translation);

      current_transformation = chainTransformations(local_transformation, current_transformation);
    }
  }

private:
  auto getPosition_(const double tangent) const -> Point override
  {
    const auto [curve_id, curve_tangent] = getCurveIdAndParameter(tangent);
    const auto curve = curves[curve_id];
    const auto transformation = transformations_[curve_id];

    return applyTransformationToPoint(curve->getPosition(curve_tangent), transformation);
  }

  auto getTangentVector_(const double tangent) const -> Vector override
  {
    const auto [curve_id, curve_tangent] = getCurveIdAndParameter(tangent);
    const auto curve = curves[curve_id];
    const auto transformation = transformations_[curve_id];

    return applyTransformationToVector(curve->getTangentVector(curve_tangent), transformation);
  }

  auto getCurveIdAndParameter(const double tangent) const -> std::pair<std::size_t, double>
  {
    validateCurveParameterOrThrow(tangent);

    const auto range_width_per_curve = 1. / curves.size();
    const auto curve_id =
      std::min(static_cast<std::size_t>(tangent / range_width_per_curve), curves.size() - 1);
    const auto curve_tangent = tangent / range_width_per_curve - curve_id;

    return std::make_pair(curve_id, curve_tangent);
  }
};  // class CombinedCurve

}  // namespace map_fragment

#endif  // MAP_FRAGMENT__PARAMETRIC_CURVE__HPP_
