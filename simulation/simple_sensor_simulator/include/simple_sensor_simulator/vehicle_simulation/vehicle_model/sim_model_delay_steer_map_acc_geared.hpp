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

#ifndef SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_DELAY_STEER_MAP_ACC_GEARED_HPP_
#define SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_DELAY_STEER_MAP_ACC_GEARED_HPP_

#include <deque>
#include <fstream>
#include <iostream>
#include <queue>
#include <simple_sensor_simulator/vehicle_simulation/vehicle_model/sim_model_interface.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/LU"

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

class AccelerationMap
{
public:
  bool readAccelerationMapFromCSV(const std::string & csv_path)
  {
    CSVLoader csv(csv_path);
    std::vector<std::vector<std::string>> table;
    if (!csv.readCSV(table)) {
      std::cerr << "[SimModelDelaySteerMapAccGeared]: failed to read acceleration map from "
                << csv_path << std::endl;
      return false;
    }

    vehicle_name_ = table[0][0];
    vel_index_ = CSVLoader::getRowIndex(table);
    acc_index_ = CSVLoader::getColumnIndex(table);
    acceleration_map_ = CSVLoader::getMap(table);

    std::cout << "[SimModelDelaySteerMapAccGeared]: success to read acceleration map from "
              << csv_path << std::endl;
    return true;
  }

  double getAcceleration(const double acc_des, const double vel) const
  {
    std::vector<double> interpolated_acc_vec;
    const double clamped_vel = CSVLoader::clampValue(vel, vel_index_, "acc: vel");

    // (throttle, vel, acc) map => (throttle, acc) map by fixing vel
    for (const auto & acc_vec : acceleration_map_) {
      interpolated_acc_vec.push_back(interpolation::lerp(vel_index_, acc_vec, clamped_vel));
    }
    // calculate throttle
    // When the desired acceleration is smaller than the throttle area, return min acc
    // When the desired acceleration is greater than the throttle area, return max acc
    const double clamped_acc = CSVLoader::clampValue(acc_des, acc_index_, "acceleration: acc");
    const double acc = interpolation::lerp(acc_index_, interpolated_acc_vec, clamped_acc);

    return acc;
  }
  std::vector<std::vector<double>> acceleration_map_;

private:
  std::string vehicle_name_;
  std::vector<double> vel_index_;
  std::vector<double> acc_index_;
};

class SimModelDelaySteerMapAccGeared : public SimModelInterface
{
public:
  /**
   * @brief constructor
   * @param [in] vx_lim velocity limit [m/s]
   * @param [in] steer_lim steering limit [rad]
   * @param [in] vx_rate_lim acceleration limit [m/ss]
   * @param [in] steer_rate_lim steering angular velocity limit [rad/ss]
   * @param [in] wheelbase vehicle wheelbase length [m]
   * @param [in] dt delta time information to set input buffer for delay
   * @param [in] acc_delay time delay for accel command [s]
   * @param [in] acc_time_constant time constant for 1D model of accel dynamics
   * @param [in] steer_delay time delay for steering command [s]
   * @param [in] steer_time_constant time constant for 1D model of steering dynamics
   * @param [in] path path to csv file for acceleration conversion map
   */
  SimModelDelaySteerMapAccGeared(
    double vx_lim, double steer_lim, double vx_rate_lim, double steer_rate_lim, double wheelbase,
    double dt, double acc_delay, double acc_time_constant, double steer_delay,
    double steer_time_constant, std::string path);

  /**
   * @brief default destructor
   */
  ~SimModelDelaySteerMapAccGeared() = default;

  AccelerationMap acc_map_;

private:
  const double MIN_TIME_CONSTANT;  //!< @brief minimum time constant

  enum IDX {
    X = 0,
    Y,
    YAW,
    VX,
    STEER,
    ACCX,
  };
  enum IDX_U {
    ACCX_DES = 0,
    STEER_DES,
    DRIVE_SHIFT,
  };

  const double vx_lim_;          //!< @brief velocity limit [m/s]
  const double vx_rate_lim_;     //!< @brief acceleration limit [m/ss]
  const double steer_lim_;       //!< @brief steering limit [rad]
  const double steer_rate_lim_;  //!< @brief steering angular velocity limit [rad/s]
  const double wheelbase_;       //!< @brief vehicle wheelbase length [m]

  std::deque<double> acc_input_queue_;    //!< @brief buffer for accel command
  std::deque<double> steer_input_queue_;  //!< @brief buffer for steering command
  const double acc_delay_;                //!< @brief time delay for accel command [s]
  const double acc_time_constant_;        //!< @brief time constant for accel dynamics
  const double steer_delay_;              //!< @brief time delay for steering command [s]
  const double steer_time_constant_;      //!< @brief time constant for steering dynamics
  const std::string path_;                //!< @brief conversion map path

  /**
   * @brief set queue buffer for input command
   * @param [in] dt delta time
   */
  void initializeInputQueue(const double & dt);

  /**
   * @brief get vehicle position x
   */
  double getX() override;

  /**
   * @brief get vehicle position y
   */
  double getY() override;

  /**
   * @brief get vehicle angle yaw
   */
  double getYaw() override;

  /**
   * @brief get vehicle velocity vx
   */
  double getVx() override;

  /**
   * @brief get vehicle lateral velocity
   */
  double getVy() override;

  /**
   * @brief get vehicle longitudinal acceleration
   */
  double getAx() override;

  /**
   * @brief get vehicle angular-velocity wz
   */
  double getWz() override;

  /**
   * @brief get vehicle steering angle
   */
  double getSteer() override;

  /**
   * @brief update vehicle states
   * @param [in] dt delta time [s]
   */
  void update(const double & dt) override;

  /**
   * @brief calculate derivative of states with time delay steering model
   * @param [in] state current model state
   * @param [in] input input vector to model
   */
  Eigen::VectorXd calcModel(const Eigen::VectorXd & state, const Eigen::VectorXd & input) override;

  /**
   * @brief update state considering current gear
   * @param [in] state current state
   * @param [in] prev_state previous state
   * @param [in] gear current gear (defined in autoware_msgs/GearCommand)
   * @param [in] dt delta time to update state
   */
  void updateStateWithGear(
    Eigen::VectorXd & state, const Eigen::VectorXd & prev_state, const uint8_t gear,
    const double dt);
};

#endif  // SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_DELAY_STEER_MAP_ACC_GEARED_HPP_
