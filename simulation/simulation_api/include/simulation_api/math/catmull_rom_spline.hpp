// Copyright 2015-2020 TierIV.inc. All rights reserved.
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
#ifndef SIMULATION_API__MATH__CATMULL_ROM_SPLINE_HPP_
#define SIMULATION_API__MATH__CATMULL_ROM_SPLINE_HPP_

#include <simulation_api/math/hermite_curve.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <vector>
#include <exception>
#include <string>

namespace simulation_api
{
namespace math
{
class SplineInterpolationError : public std::runtime_error
{
public:
  explicit SplineInterpolationError(const char * message)
  : runtime_error(message) {}
  explicit SplineInterpolationError(std::string message)
  : runtime_error(message.c_str()) {}
};

class CatmullRomSpline
{
public:
  explicit CatmullRomSpline(std::vector<geometry_msgs::msg::Point> control_points);
  double getLength() const {return total_length_;}

private:
  bool checkConnection() const;
  bool equals(geometry_msgs::msg::Point p0, geometry_msgs::msg::Point p1) const;
  std::vector<HermiteCurve> curves_;
  std::vector<double> length_list_;
  double total_length_;
  const std::vector<geometry_msgs::msg::Point> control_points;
};
}  // namespace math
}  // namespace simulation_api

#endif  // SIMULATION_API__MATH__CATMULL_ROM_SPLINE_HPP_
