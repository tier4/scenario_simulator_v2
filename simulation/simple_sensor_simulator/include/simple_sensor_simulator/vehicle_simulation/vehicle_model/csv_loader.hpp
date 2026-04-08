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

#ifndef SIMPLE_SENSOR_SIMULATOR__VEHICLE_MODEL__CSV_LOADER_HPP_
#define SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__CSV_LOADER_HPP_

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

/// @note copied from https://github.com/tier4/autoware.universe/blob/v0.17.0/common/interpolation/include/interpolation/interpolation_utils.hpp
namespace interpolation_utils
{
inline bool isIncreasing(const std::vector<double> & x)
{
  if (x.empty()) {
    throw std::invalid_argument("Points is empty.");
  }

  for (size_t i = 0; i < x.size() - 1; ++i) {
    if (x.at(i) >= x.at(i + 1)) {
      return false;
    }
  }

  return true;
}

inline bool isNotDecreasing(const std::vector<double> & x)
{
  if (x.empty()) {
    throw std::invalid_argument("Points is empty.");
  }

  for (size_t i = 0; i < x.size() - 1; ++i) {
    if (x.at(i) > x.at(i + 1)) {
      return false;
    }
  }

  return true;
}

inline std::vector<double> validateKeys(
  const std::vector<double> & base_keys, const std::vector<double> & query_keys)
{
  // when vectors are empty
  if (base_keys.empty() || query_keys.empty()) {
    throw std::invalid_argument("Points is empty.");
  }

  // when size of vectors are less than 2
  if (base_keys.size() < 2) {
    throw std::invalid_argument(
      "The size of points is less than 2. base_keys.size() = " + std::to_string(base_keys.size()));
  }

  // when indices are not sorted
  if (!isIncreasing(base_keys) || !isNotDecreasing(query_keys)) {
    throw std::invalid_argument("Either base_keys or query_keys is not sorted.");
  }

  // when query_keys is out of base_keys (This function does not allow exterior division.)
  constexpr double epsilon = 1e-3;
  if (
    query_keys.front() < base_keys.front() - epsilon ||
    base_keys.back() + epsilon < query_keys.back()) {
    throw std::invalid_argument("query_keys is out of base_keys");
  }

  // NOTE: Due to calculation error of double, a query key may be slightly out of base keys.
  //       Therefore, query keys are cropped here.
  auto validated_query_keys = query_keys;
  validated_query_keys.front() = std::max(validated_query_keys.front(), base_keys.front());
  validated_query_keys.back() = std::min(validated_query_keys.back(), base_keys.back());

  return validated_query_keys;
}

template <class T>
void validateKeysAndValues(
  const std::vector<double> & base_keys, const std::vector<T> & base_values)
{
  // when vectors are empty
  if (base_keys.empty() || base_values.empty()) {
    throw std::invalid_argument("Points is empty.");
  }

  // when size of vectors are less than 2
  if (base_keys.size() < 2 || base_values.size() < 2) {
    throw std::invalid_argument(
      "The size of points is less than 2. base_keys.size() = " + std::to_string(base_keys.size()) +
      ", base_values.size() = " + std::to_string(base_values.size()));
  }

  // when sizes of indices and values are not same
  if (base_keys.size() != base_values.size()) {
    throw std::invalid_argument("The size of base_keys and base_values are not the same.");
  }
}
}  // namespace interpolation_utils

/// @note copied from https://github.com/tier4/autoware.universe/blob/v0.17.0/common/interpolation/include/interpolation/linear_interpolation.hpp
namespace interpolation
{
double lerp(const double src_val, const double dst_val, const double ratio);

std::vector<double> lerp(
  const std::vector<double> & base_keys, const std::vector<double> & base_values,
  const std::vector<double> & query_keys);

double lerp(
  const std::vector<double> & base_keys, const std::vector<double> & base_values,
  const double query_key);

}  // namespace interpolation

using Table = std::vector<std::vector<std::string>>;
using Map = std::vector<std::vector<double>>;

/// @note copied from https://github.com/autowarefoundation/autoware.universe/blob/main/simulator/simple_planning_simulator/include/simple_planning_simulator/utils/csv_loader.hpp
class CSVLoader
{
public:
  explicit CSVLoader(const std::string & csv_path);

  bool readCSV(Table & result, const char delim = ',');
  static bool validateData(const Table & table, const std::string & csv_path);
  static bool validateMap(const Map & map, const bool is_col_decent);
  static Map getMap(const Table & table);
  static std::vector<double> getRowIndex(const Table & table);
  static std::vector<double> getColumnIndex(const Table & table);
  static double clampValue(
    const double val, const std::vector<double> & ranges, const std::string & name);

private:
  std::string csv_path_;
};

#endif  // SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__CSV_LOADER_HPP_
