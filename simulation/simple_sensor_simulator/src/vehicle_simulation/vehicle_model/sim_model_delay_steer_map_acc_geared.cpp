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
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <simple_sensor_simulator/vehicle_simulation/vehicle_model/sim_model_delay_steer_map_acc_geared.hpp>

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
    std::cerr << "Cannot open " << csv_path_.c_str() << std::endl;
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
    std::cerr << "index around (i,j) is invalid ( " << invalid_index_pair.first << ", "
              << invalid_index_pair.second << " )" << std::endl;
    return false;
  }
  return true;
}

bool CSVLoader::validateData(const Table & table, const std::string & csv_path)
{
  if (table.empty()) {
    std::cerr << "The table is empty." << std::endl;
    return false;
  }
  if (table[0].size() < 2) {
    std::cerr << "Cannot read " << csv_path.c_str() << " CSV file should have at least 2 column"
              << std::endl;
    return false;
  }
  // validate map size
  for (size_t i = 1; i < table.size(); i++) {
    // validate row size
    if (table[0].size() != table[i].size()) {
      std::cerr << "Cannot read " << csv_path.c_str()
                << ". Each row should have a same number of columns" << std::endl;
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
    std::cerr << "Input " << name << ": " << val
              << " is out of range. use closest value. Please update the conversion map"
              << std::endl;
    return std::min(std::max(val, min_value), max_value);
  }
  return val;
}

SimModelDelaySteerMapAccGeared::SimModelDelaySteerMapAccGeared(
  double vx_lim, double steer_lim, double vx_rate_lim, double steer_rate_lim, double wheelbase,
  double dt, double acc_delay, double acc_time_constant, double steer_delay,
  double steer_time_constant, std::string path)
: SimModelInterface(6 /* dim x */, 2 /* dim u */),
  MIN_TIME_CONSTANT(0.03),
  vx_lim_(vx_lim),
  vx_rate_lim_(vx_rate_lim),
  steer_lim_(steer_lim),
  steer_rate_lim_(steer_rate_lim),
  wheelbase_(wheelbase),
  acc_delay_(acc_delay),
  acc_time_constant_(std::max(acc_time_constant, MIN_TIME_CONSTANT)),
  steer_delay_(steer_delay),
  steer_time_constant_(std::max(steer_time_constant, MIN_TIME_CONSTANT)),
  path_(path)
{
  initializeInputQueue(dt);
  acc_map_.readAccelerationMapFromCSV(path_);
}

double SimModelDelaySteerMapAccGeared::getX() { return state_(IDX::X); }
double SimModelDelaySteerMapAccGeared::getY() { return state_(IDX::Y); }
double SimModelDelaySteerMapAccGeared::getYaw() { return state_(IDX::YAW); }
double SimModelDelaySteerMapAccGeared::getVx() { return state_(IDX::VX); }
double SimModelDelaySteerMapAccGeared::getVy() { return 0.0; }
double SimModelDelaySteerMapAccGeared::getAx() { return state_(IDX::ACCX); }
double SimModelDelaySteerMapAccGeared::getWz()
{
  return state_(IDX::VX) * std::tan(state_(IDX::STEER)) / wheelbase_;
}
double SimModelDelaySteerMapAccGeared::getSteer() { return state_(IDX::STEER); }
void SimModelDelaySteerMapAccGeared::update(const double & dt)
{
  Eigen::VectorXd delayed_input = Eigen::VectorXd::Zero(dim_u_);

  acc_input_queue_.push_back(input_(IDX_U::ACCX_DES));
  delayed_input(IDX_U::ACCX_DES) = acc_input_queue_.front();
  acc_input_queue_.pop_front();
  steer_input_queue_.push_back(input_(IDX_U::STEER_DES));
  delayed_input(IDX_U::STEER_DES) = steer_input_queue_.front();
  steer_input_queue_.pop_front();

  const auto prev_state = state_;
  updateRungeKutta(dt, delayed_input);

  // take velocity limit explicitly
  state_(IDX::VX) = std::max(-vx_lim_, std::min(state_(IDX::VX), vx_lim_));

  // consider gear
  // update position and velocity first, and then acceleration is calculated naturally
  updateStateWithGear(state_, prev_state, gear_, dt);
}

void SimModelDelaySteerMapAccGeared::initializeInputQueue(const double & dt)
{
  size_t acc_input_queue_size = static_cast<size_t>(round(acc_delay_ / dt));
  acc_input_queue_.resize(acc_input_queue_size);
  std::fill(acc_input_queue_.begin(), acc_input_queue_.end(), 0.0);

  size_t steer_input_queue_size = static_cast<size_t>(round(steer_delay_ / dt));
  steer_input_queue_.resize(steer_input_queue_size);
  std::fill(steer_input_queue_.begin(), steer_input_queue_.end(), 0.0);
}

Eigen::VectorXd SimModelDelaySteerMapAccGeared::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  const double vel = std::clamp(state(IDX::VX), -vx_lim_, vx_lim_);
  const double acc = std::clamp(state(IDX::ACCX), -vx_rate_lim_, vx_rate_lim_);
  const double yaw = state(IDX::YAW);
  const double steer = state(IDX::STEER);
  const double acc_des = std::clamp(input(IDX_U::ACCX_DES), -vx_rate_lim_, vx_rate_lim_);
  const double steer_des = std::clamp(input(IDX_U::STEER_DES), -steer_lim_, steer_lim_);
  double steer_rate = -(steer - steer_des) / steer_time_constant_;
  steer_rate = std::clamp(steer_rate, -steer_rate_lim_, steer_rate_lim_);

  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
  d_state(IDX::X) = vel * cos(yaw);
  d_state(IDX::Y) = vel * sin(yaw);
  d_state(IDX::YAW) = vel * std::tan(steer) / wheelbase_;
  d_state(IDX::VX) = acc;
  d_state(IDX::STEER) = steer_rate;
  const double converted_acc =
    acc_map_.acceleration_map_.empty() ? acc_des : acc_map_.getAcceleration(acc_des, vel);

  d_state(IDX::ACCX) = -(acc - converted_acc) / acc_time_constant_;

  return d_state;
}

void SimModelDelaySteerMapAccGeared::updateStateWithGear(
  Eigen::VectorXd & state, const Eigen::VectorXd & prev_state, const uint8_t gear, const double dt)
{
  using autoware_vehicle_msgs::msg::GearCommand;
  if (
    gear == GearCommand::DRIVE || gear == GearCommand::DRIVE_2 || gear == GearCommand::DRIVE_3 ||
    gear == GearCommand::DRIVE_4 || gear == GearCommand::DRIVE_5 || gear == GearCommand::DRIVE_6 ||
    gear == GearCommand::DRIVE_7 || gear == GearCommand::DRIVE_8 || gear == GearCommand::DRIVE_9 ||
    gear == GearCommand::DRIVE_10 || gear == GearCommand::DRIVE_11 ||
    gear == GearCommand::DRIVE_12 || gear == GearCommand::DRIVE_13 ||
    gear == GearCommand::DRIVE_14 || gear == GearCommand::DRIVE_15 ||
    gear == GearCommand::DRIVE_16 || gear == GearCommand::DRIVE_17 ||
    gear == GearCommand::DRIVE_18 || gear == GearCommand::LOW || gear == GearCommand::LOW_2) {
    if (state(IDX::VX) < 0.0) {
      state(IDX::VX) = 0.0;
      state(IDX::X) = prev_state(IDX::X);
      state(IDX::Y) = prev_state(IDX::Y);
      state(IDX::YAW) = prev_state(IDX::YAW);
      state(IDX::ACCX) = (state(IDX::VX) - prev_state(IDX::VX)) / std::max(dt, 1.0e-5);
    }
  } else if (gear == GearCommand::REVERSE || gear == GearCommand::REVERSE_2) {
    if (state(IDX::VX) > 0.0) {
      state(IDX::VX) = 0.0;
      state(IDX::X) = prev_state(IDX::X);
      state(IDX::Y) = prev_state(IDX::Y);
      state(IDX::YAW) = prev_state(IDX::YAW);
      state(IDX::ACCX) = (state(IDX::VX) - prev_state(IDX::VX)) / std::max(dt, 1.0e-5);
    }
  } else if (gear == GearCommand::PARK) {
    state(IDX::VX) = 0.0;
    state(IDX::X) = prev_state(IDX::X);
    state(IDX::Y) = prev_state(IDX::Y);
    state(IDX::YAW) = prev_state(IDX::YAW);
    state(IDX::ACCX) = (state(IDX::VX) - prev_state(IDX::VX)) / std::max(dt, 1.0e-5);
  } else {
    state(IDX::VX) = 0.0;
    state(IDX::X) = prev_state(IDX::X);
    state(IDX::Y) = prev_state(IDX::Y);
    state(IDX::YAW) = prev_state(IDX::YAW);
    state(IDX::ACCX) = (state(IDX::VX) - prev_state(IDX::VX)) / std::max(dt, 1.0e-5);
  }
}
