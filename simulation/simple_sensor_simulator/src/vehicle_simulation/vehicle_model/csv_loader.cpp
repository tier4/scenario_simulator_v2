// Copyright 2023 The Autoware Foundation.
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

#include <algorithm>
#include <rclcpp/logging.hpp>
#include <simple_sensor_simulator/vehicle_simulation/vehicle_model/csv_loader.hpp>
#include <string>
#include <vector>

// copied from https://github.com/tier4/autoware.universe/blob/v0.17.0/common/interpolation/src/linear_interpolation.cpp
namespace interpolation
{
double lerp(const double src_val, const double dst_val, const double ratio)
{
  return src_val + (dst_val - src_val) * ratio;
}

std::vector<double> lerp(
  const std::vector<double> & base_keys, const std::vector<double> & base_values,
  const std::vector<double> & query_keys)
{
  // throw exception for invalid arguments
  const auto validated_query_keys = interpolation_utils::validateKeys(base_keys, query_keys);
  interpolation_utils::validateKeysAndValues(base_keys, base_values);

  // calculate linear interpolation
  std::vector<double> query_values;
  size_t key_index = 0;
  for (const auto query_key : validated_query_keys) {
    while (base_keys.at(key_index + 1) < query_key) {
      ++key_index;
    }

    const double src_val = base_values.at(key_index);
    const double dst_val = base_values.at(key_index + 1);
    const double ratio = (query_key - base_keys.at(key_index)) /
                         (base_keys.at(key_index + 1) - base_keys.at(key_index));

    const double interpolated_val = lerp(src_val, dst_val, ratio);
    query_values.push_back(interpolated_val);
  }

  return query_values;
}

double lerp(
  const std::vector<double> & base_keys, const std::vector<double> & base_values, double query_key)
{
  return lerp(base_keys, base_values, std::vector<double>{query_key}).front();
}
}  // namespace interpolation

CSVLoader::CSVLoader(const std::string & csv_path) { csv_path_ = csv_path; }

bool CSVLoader::readCSV(Table & result, const char delim)
{
  std::ifstream ifs(csv_path_);
  if (!ifs.is_open()) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("CSVLoader"), "Cannot open " << csv_path_.c_str());
    return false;
  }

  std::string buf;
  while (std::getline(ifs, buf)) {
    std::vector<std::string> tokens;

    std::istringstream iss(buf);
    std::string token;
    while (std::getline(iss, token, delim)) {
      tokens.push_back(token);
    }

    if (tokens.size() != 0) {
      result.push_back(tokens);
    }
  }
  if (!validateData(result, csv_path_)) {
    return false;
  }
  return true;
}

bool CSVLoader::validateMap(const Map & map, const bool is_col_decent)
{
  std::pair<size_t, size_t> invalid_index_pair;
  bool is_invalid = false;
  // validate interpolation
  for (size_t i = 1; i < map.size(); i++) {
    const auto & vec = map.at(i);
    const auto & prev_vec = map.at(i - 1);
    // validate row data
    for (size_t j = 0; j < vec.size(); j++) {
      // validate col
      if (vec.at(j) <= prev_vec.at(j) && is_col_decent) {
        invalid_index_pair = std::make_pair(i, j);
        is_invalid = true;
      }
      if (vec.at(j) >= prev_vec.at(j) && !is_col_decent) {
        invalid_index_pair = std::make_pair(i, j);
        is_invalid = true;
      }
    }
  }
  if (is_invalid) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("CSVLoader"), "index around (i,j) is invalid ( "
                                         << invalid_index_pair.first << ", "
                                         << invalid_index_pair.second << " )");
    return false;
  }
  return true;
}

bool CSVLoader::validateData(const Table & table, const std::string & csv_path)
{
  if (table.empty()) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("CSVLoader"), "The table is empty.");
    return false;
  }
  if (table[0].size() < 2) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("CSVLoader"),
      "Cannot read " << csv_path.c_str() << " CSV file should have at least 2 column");
    return false;
  }
  // validate map size
  for (size_t i = 1; i < table.size(); i++) {
    // validate row size
    if (table[0].size() != table[i].size()) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("CSVLoader"),
        "Cannot read " << csv_path.c_str() << ". Each row should have a same number of columns");
      return false;
    }
  }
  return true;
}

Map CSVLoader::getMap(const Table & table)
{
  Map map = {};
  for (size_t i = 1; i < table.size(); i++) {
    std::vector<double> accelerations;
    for (size_t j = 1; j < table[i].size(); j++) {
      accelerations.push_back(std::stod(table[i][j]));
    }
    map.push_back(accelerations);
  }
  return map;
}

std::vector<double> CSVLoader::getRowIndex(const Table & table)
{
  std::vector<double> index = {};
  for (size_t i = 1; i < table[0].size(); i++) {
    index.push_back(std::stod(table[0][i]));
  }
  return index;
}

std::vector<double> CSVLoader::getColumnIndex(const Table & table)
{
  std::vector<double> index = {};
  for (size_t i = 1; i < table.size(); i++) {
    index.push_back(std::stod(table[i][0]));
  }
  return index;
}

double CSVLoader::clampValue(
  const double val, const std::vector<double> & ranges, const std::string & name)
{
  const double max_value = *std::max_element(ranges.begin(), ranges.end());
  const double min_value = *std::min_element(ranges.begin(), ranges.end());
  if (val < min_value || max_value < val) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("CSVLoader"),
      "Input " << name << ": " << val
               << " is out of range. use closest value. Please update the conversion map");
    return std::min(std::max(val, min_value), max_value);
  }
  return val;
}
