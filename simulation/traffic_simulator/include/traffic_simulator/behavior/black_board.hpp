// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#ifndef TRAFFIC_SIMULATOR__BEHAVIOR__BLACK_BOARD_HPP_
#define TRAFFIC_SIMULATOR__BEHAVIOR__BLACK_BOARD_HPP_

#include <boost/any.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <string>
#include <unordered_map>

namespace entity_behavior
{
class BlackBoard
{
public:
  template <typename T>
  void set(const std::string & key, const T & value)
  {
    data_[key] = boost::any_cast<T>(value);
  }
  template <typename T>
  void get(const std::string & key, T & value) const
  {
    try {
      value = boost::any_cast<T>(getValue(key));
    } catch (const boost::bad_any_cast & e) {
      THROW_SIMULATION_ERROR("value : ", key, " is not specified type.");
    }
  }
  template <typename T>
  void get(const std::string & key, T & value, const bool & is_empty) const
  {
    try {
      value = boost::any_cast<T>(getValue(key));
    } catch (const boost::bad_any_cast & e) {
      is_empty = true;
      return;
    }
    is_empty = true;
  }

private:
  boost::any getValue(const std::string & key) const;
  std::unordered_map<std::string, boost::any> data_;
};
}  // namespace entity_behavior

#endif  // TRAFFIC_SIMULATOR__BEHAVIOR__BLACK_BOARD_HPP_
