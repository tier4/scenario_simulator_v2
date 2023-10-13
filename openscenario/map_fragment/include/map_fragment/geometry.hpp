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

#include <boost/geometry/geometries/point_xy.hpp>
#include <cmath>
#include <string>

namespace map_fragment
{

using Point2d = boost::geometry::model::d2::point_xy<double>;
using Vector2d = boost::geometry::model::d2::point_xy<double>;
using UnitVector2d = Vector2d;  // TODO

Point2d operator+(Point2d a, Vector2d const& b)
{
  a.x(a.x() + b.x());
  a.y(a.y() + b.y());
  return a;
}

Vector2d operator*(Vector2d v, double const& multiplier)
{
  v.x(v.x() * multiplier);
  v.y(v.y() * multiplier);
  return v;
}

Vector2d operator-(Point2d const& a, Point2d const& b)
{
  return Vector2d(a.x() - b.x(),
                  a.y() - b.y());
}

Vector2d rotate(Vector2d v, double angle) {
  return Vector2d(v.x() * std::cos(angle) - v.y() * std::sin(angle),
                  v.x() * std::sin(angle) + v.y() * std::cos(angle));
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
  using Ptr = std::shared_ptr<ParametricCurve>;
  
  /**
   * Calculate curve point for given parameter t ∈ [0, 1]
   */
  Point2d getPosition(double t)
  {
    validateCurveParameterOrThrow(t);
    return getPosition_(t);
  }

  /**
   * Calculate curve tangent vector for given parameter t ∈ [0, 1]
   */
  UnitVector2d getTangentVector(double t)
  {
    validateCurveParameterOrThrow(t);

    // TODO Throw error if the vector is not unit?
    return getTangentVector_(t);
  }

private:
  virtual Point2d getPosition_(double t) = 0;
  virtual UnitVector2d getTangentVector_(double t) = 0;

  void validateCurveParameterOrThrow(double t)
  {
    if (!(0 <= t && t <= 1)) {
      throw std::invalid_argument(
        "Expected t to be in range [0, 1]. Actual value: "
        + std::to_string(t)
      );
    }
  }
};  // class ParametricCurve

/**
 * Straight line of given length
 */
class Straight : public ParametricCurve
{
  double length_;

public:
  Straight(double length)
    : length_(length)
  {
    if (length <= 0.0) {
      throw std::invalid_argument(
        "Expected length to be positive. Actual value: "
        + std::to_string(length)
      );
    }
  }

private:
  virtual Point2d getPosition_(double t) override
  {
    return Point2d(t * length_, 0);
  }
  
  virtual UnitVector2d getTangentVector_(double) override
  {
    return UnitVector2d(1, 0);
  }
};  // class Straight

/**
 * Arc of given radius and angle (positive to the left)
 */
class Arc : public ParametricCurve
{
  double radius_;
  double angle_;
  Point2d center_;

public:
  Arc(double radius, double angle)
    : radius_(radius), angle_(angle)
  {
    if (radius <= 0.0)
    {
      throw std::invalid_argument(
        "Expected radius to be positive. Actual value:"
        + std::to_string(radius)
      );
    }

    if (!(0 < abs(angle) && abs(angle) < 2 * M_PI))
    {
      throw std::invalid_argument(
        "Expected angle to be in range (0, 2 * PI) or (-2 * PI, 0). Actual value: "
        + std::to_string(angle)
      );
    }

    if (angle > 0)
    {
      center_ = Point2d(0, radius);
    }

    else
    {
      center_ = Point2d(0, -radius);
    }
  }
  
private:
  virtual Point2d getPosition_(double t) override
  {
    auto theta = t * angle_;
    return center_ + rotate(Point2d(0, 0) - center_, theta);
  }
  
  virtual UnitVector2d getTangentVector_(double t) override
  {
    auto theta = t * angle_;
    return rotate(Vector2d(1, 0), theta);
  }
};  // class Arc

}  // namespace map_fragment

#endif  // MAP_FRAGMENT__GEOMETRY__HPP_