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

#include <simulation_interface/simulation_api_schema.pb.h>
#include <simulation_interface/operators.hpp>

#include <boost/math/constants/constants.hpp>
#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <execution>
#include <get_parameter/get_parameter.hpp>
#include <regex>
#include <scenario_simulator_exception/exception.hpp>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace simple_sensor_simulator
{
namespace noise_parameter_selector
{
inline auto createEllipticalParameterSelector(const std::string & parameter_base_path, double x, double y)
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

// Extract config name from parameter name given a version base path
// Returns empty string if extraction fails
inline auto parseConfigNameFromParameter(
  const std::string & parameter_name, const std::string & version_base_path) -> std::string
{
  if (const auto next_dot_pos = parameter_name.find('.', version_base_path.length());
      next_dot_pos != std::string::npos) {
    return parameter_name.substr(
      version_base_path.length(), next_dot_pos - version_base_path.length());
  }
  return "";
}

// Get all noise config names for a given topic and version
inline auto listAvailableNoiseConfigs(const std::string & topic_name, const std::string & version)
  -> std::vector<std::string>
{
  const std::string version_base_path = topic_name + ".noise." + version + ".";

  // Get all parameter names
  const auto parameter_names =
    common::getParameterNode()
      .list_parameters({}, rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE)
      .names;

  // Extract unique config names from parameters
  std::set<std::string> config_names_set;
  for (const auto & parameter_name : parameter_names) {
    if (parameter_name.rfind(version_base_path, 0) == 0) {
      const auto config_name = parseConfigNameFromParameter(parameter_name, version_base_path);
      if (!config_name.empty()) {
        config_names_set.insert(config_name);
      }
    }
  }

  return std::vector<std::string>(config_names_set.begin(), config_names_set.end());
}

inline auto findMatchingNoiseConfigForEntity(
  const traffic_simulator_msgs::EntityStatus & entity, const std::string & version,
  const std::string & topic_name) -> std::string
{
  const std::string version_base_path = topic_name + ".noise." + version + ".";
  auto matches_noise_application_entities = [&](
                                              const traffic_simulator_msgs::EntityStatus & entity,
                                              const std::string & noise_config_name) -> bool {
    const auto base_path = version_base_path + noise_config_name + ".noise_application_entities.";

    const auto types = common::getParameter<std::vector<std::string>>(base_path + "types");
    const auto subtypes = common::getParameter<std::vector<std::string>>(base_path + "subtypes");
    const auto names = common::getParameter<std::vector<std::string>>(base_path + "names");

    auto to_string = [](const auto & obj) {
      std::ostringstream oss;
      oss << obj;
      return oss.str();
    };

    auto string_with_wildcards_to_regex =
      [](const std::string & string_with_wildcards) -> std::regex {
      std::string regex_pattern;
      for (char c : string_with_wildcards) {
        regex_pattern += (c == '*') ? ".*" : (c == '?') ? "." : std::string(1, c);
      }
      return std::regex(regex_pattern);
    };

    return std::any_of(
             types.begin(), types.end(),
             [entity_type = to_string(entity.type()),
              string_with_wildcards_to_regex](const auto & target) {
               return std::regex_match(entity_type, string_with_wildcards_to_regex(target));
             }) &&
           std::any_of(
             subtypes.begin(), subtypes.end(),
             [entity_subtype = to_string(entity.subtype()),
              string_with_wildcards_to_regex](const auto & target) {
               return std::regex_match(entity_subtype, string_with_wildcards_to_regex(target));
             }) &&
           std::any_of(
             names.begin(), names.end(),
             [entity_name = entity.name(), string_with_wildcards_to_regex](const auto & target) {
               return std::regex_match(entity_name, string_with_wildcards_to_regex(target));
             });
  };

  const auto parameter_names =
    common::getParameterNode()
      .list_parameters({}, rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE)
      .names;

  if (auto matched_parameter = std::find_if(
        parameter_names.begin(), parameter_names.end(),
        [&](const auto & parameter_name) {
          if (parameter_name.rfind(version_base_path, 0) == 0) {
            if (auto config_name = parseConfigNameFromParameter(parameter_name, version_base_path);
                !config_name.empty()) {
              return matches_noise_application_entities(entity, config_name);
            }
          }
          return false;
        });
      matched_parameter != parameter_names.end()) {
    return parseConfigNameFromParameter(*matched_parameter, version_base_path);
  } else {
    return "";
  }
}

}  // namespace noise_parameter_selector
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__NOISE_PARAMETER_SELECTOR_HPP_
