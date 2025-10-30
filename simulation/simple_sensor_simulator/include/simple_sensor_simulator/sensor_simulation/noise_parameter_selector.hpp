// Copyright 2015 TIER IV, Inc. All rights reserved.
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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__NOISE_PARAMETER_SELECTOR_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__NOISE_PARAMETER_SELECTOR_HPP_

#include <boost/math/constants/constants.hpp>
#include <execution>
#include <get_parameter/get_parameter.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <simulation_interface/operators.hpp>
#include <string>
#include <vector>

namespace simple_sensor_simulator
{
namespace noise_parameter_selector
{
inline auto createEllipticalParameterSelector(
  const std::string & parameter_base_path, double x, double y)
{
  return [parameter_base_path, x, y](const std::string & name) {
    return [=]() {
      const auto ellipse_y_radii =
        common::getParameter<std::vector<double>>(parameter_base_path + "ellipse_y_radii");
      const auto ellipse_normalized_x_radius =
        common::getParameter<double>(parameter_base_path + name + ".ellipse_normalized_x_radius");
      const auto values =
        common::getParameter<std::vector<double>>(parameter_base_path + name + ".values");
      if (ellipse_y_radii.size() == values.size()) {
        /*
           If the parameter `ellipse_y_radii` contains the value 0.0,
           division by zero will occur here.
           However, in that case, the distance will be NaN, which correctly
           expresses the meaning that "the distance cannot be defined", and
           this function will work without any problems (zero will be
           returned).
        */
        const auto distance = std::hypot(x / ellipse_normalized_x_radius, y);
        for (auto i = std::size_t(0); i < ellipse_y_radii.size(); ++i) {
          if (distance < ellipse_y_radii[i]) {
            return values[i];
          }
        }
        return 0.0;
      } else {
        throw common::Error(
          "Array size mismatch: ", std::quoted(parameter_base_path + "ellipse_y_radii"), " has ",
          ellipse_y_radii.size(), " elements, but ",
          std::quoted(parameter_base_path + name + ".values"), " has ", values.size(),
          " elements. Both arrays must have the same size.");
      }
    };
  };
}
}  // namespace noise_parameter_selector
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__NOISE_PARAMETER_SELECTOR_HPP_
