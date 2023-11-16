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

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

namespace map_fragment
{

struct Point2d
{
  double x = 0;
  double y = 0;
};

struct Vector2d
{
  double x = 0;
  double y = 0;
};

auto operator+(Point2d const & a, Vector2d const & b) -> Point2d { return {a.x + b.x, a.y + b.y}; }

auto operator*(Vector2d const & v, double multiplier) -> Vector2d
{
  return {v.x * multiplier, v.y * multiplier};
}

auto operator-(Point2d const & a, Point2d const & b) -> Vector2d { return {a.x - b.x, a.y - b.y}; }

auto operator+(Vector2d const & a, Vector2d const & b) -> Vector2d
{
  return {a.x + b.x, a.y + b.y};
}

auto rotate(Vector2d const & v, double angle) -> Vector2d
{
  return {
    v.x * std::cos(angle) - v.y * std::sin(angle), v.x * std::sin(angle) + v.y * std::cos(angle)};
}

auto rotate(Point2d const & p, double angle) -> Point2d
{
  Point2d p0 = {0, 0};
  return p0 + rotate(p - p0, angle);
}

auto calculateAngle(Vector2d const & v) -> double { return std::atan2(v.y, v.x); }

struct Transformation2d
{
  Vector2d translation;
  double rotation;
};  // struct Transformation2d

auto applyTransformation(Point2d const & p, Transformation2d const & t) -> Point2d
{
  return rotate(p, t.rotation) + t.translation;
}

auto applyTransformation(Vector2d const & v, Transformation2d const & t) -> Vector2d
{
  return rotate(v, t.rotation);
}

auto chainTransformations(Transformation2d const & first, Transformation2d const & second)
  -> Transformation2d
{
  Transformation2d t;
  t.translation = first.translation + rotate(second.translation, first.rotation);
  t.rotation = first.rotation + second.rotation;
  return t;
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
   * Calculate curve point for given parameter t ∈ [0, 1]
   */
  auto getPosition(double t) -> Point2d
  {
    validateCurveParameterOrThrow(t);
    return getPosition_(t);
  }

  /**
   * Calculate curve tangent vector for given parameter t ∈ [0, 1]
   */
  auto getTangentVector(double t) -> Vector2d
  {
    validateCurveParameterOrThrow(t);

    // TODO Throw error if the vector is not unit?
    return getTangentVector_(t);
  }

private:
  virtual auto getPosition_(double t) -> Point2d = 0;
  virtual auto getTangentVector_(double t) -> Vector2d = 0;

protected:
  auto validateCurveParameterOrThrow(double t) -> void
  {
    if (!(0 <= t && t <= 1)) {
      throw std::invalid_argument(
        "Expected t to be in range [0, 1]. Actual value: " + std::to_string(t));
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
  auto getPosition_(double t) -> Point2d override { return {t * length, 0}; }

  auto getTangentVector_(double) -> Vector2d override { return {1, 0}; }
};  // class Straight

/**
 * Arc of given radius and angle (positive to the left)
 */
class Arc : public ParametricCurve
{
  Point2d center_;

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

    center_.y = angle > 0 ? radius : -radius;
  }

private:
  auto getPosition_(double t) -> Point2d override
  {
    auto theta = t * angle;
    Point2d origin = {0, 0};
    return center_ + rotate(origin - center_, theta);
  }

  auto getTangentVector_(double t) -> Vector2d override
  {
    auto theta = t * angle;
    Vector2d v = {1, 0};
    return rotate(v, theta);
  }
};  // class Arc

/**
 * Combination of two or more component curves, chained after one another
 */
class CombinedCurve : public ParametricCurve
{
  std::vector<Transformation2d> transformations_;

public:
  const std::vector<ParametricCurve::Pointer> curves;

  explicit CombinedCurve(std::vector<ParametricCurve::Pointer> const & curves) : curves(curves)
  {
    if (curves.size() < 2) {
      throw std::invalid_argument(
        "Number of curves to be combined must be two or more. Actual number: " +
        std::to_string(curves.size()));
    }

    Transformation2d current_transformation;

    for (const auto & curve : curves) {
      transformations_.push_back(current_transformation);

      Point2d origin = {0, 0};
      Transformation2d local_transformation = {
        curve->getPosition(1.0) - origin, calculateAngle(curve->getTangentVector(1.0))};

      current_transformation = chainTransformations(current_transformation, local_transformation);
    }
  }

private:
  auto getPosition_(double t) -> Point2d override
  {
    auto [curve_id, curve_t] = getCurveIdAndParameter(t);
    auto curve = curves[curve_id];
    auto transformation = transformations_[curve_id];

    return applyTransformation(curve->getPosition(curve_t), transformation);
  }

  auto getTangentVector_(double t) -> Vector2d override
  {
    auto [curve_id, curve_t] = getCurveIdAndParameter(t);
    auto curve = curves[curve_id];
    auto transformation = transformations_[curve_id];

    return applyTransformation(curve->getTangentVector(curve_t), transformation);
  }

  auto getCurveIdAndParameter(double t) -> std::pair<std::size_t, double>
  {
    validateCurveParameterOrThrow(t);

    auto range_width_per_curve = 1. / curves.size();
    auto curve_id =
      std::min(static_cast<std::size_t>(t / range_width_per_curve), curves.size() - 1);
    auto curve_t = t / range_width_per_curve - curve_id;

    return std::make_pair(curve_id, curve_t);
  }
};  // class CombinedCurve

}  // namespace map_fragment

#endif  // MAP_FRAGMENT__GEOMETRY__HPP_
