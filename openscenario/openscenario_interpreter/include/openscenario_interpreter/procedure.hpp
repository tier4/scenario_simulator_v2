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

#ifndef OPENSCENARIO_INTERPRETER__PROCEDURE_HPP_
#define OPENSCENARIO_INTERPRETER__PROCEDURE_HPP_

#include <simulation_api/api/api.hpp>
#include <openscenario_interpreter/error.hpp>

#include <limits>
#include <memory>
#include <utility>

namespace openscenario_interpreter
{
extern scenario_simulator::API & connection;

// static struct Connector
// {
//   Connector();
//   ~Connector();
// } connector;

template<typename ... Ts>
decltype(auto) connect(Ts && ... xs)
{
  new (&connection) scenario_simulator::API(std::forward<decltype(xs)>(xs)...);
  return connection;
}

template<typename ... Ts>
decltype(auto) initialize(Ts && ... xs)
{
  return connection.initialize(std::forward<decltype(xs)>(xs)...);
}

template<typename ... Ts>
decltype(auto) spawn(Ts && ... xs)
{
  return connection.spawn(std::forward<decltype(xs)>(xs)...);
}

template<typename ... Ts>
decltype(auto) requestLaneChange(Ts && ... xs)
{
  return connection.requestLaneChange(std::forward<decltype(xs)>(xs)...);
}

template<typename ... Ts>
decltype(auto) isInLanelet(Ts && ... xs)
{
  return connection.isInLanelet(std::forward<decltype(xs)>(xs)...);
}

template<typename ... Ts>
decltype(auto) setTargetSpeed(Ts && ... xs)
{
  return connection.setTargetSpeed(std::forward<decltype(xs)>(xs)...);
}

template<typename ... Ts>
decltype(auto) getEntityStatus(Ts && ... xs) try {
  return connection.getEntityStatus(std::forward<decltype(xs)>(xs)...);
} catch (const simulation_api::SimulationRuntimeError & error) {
  std::stringstream ss {};
  ss << error.what() << ".\n";
  ss << "Possible causes:\n";
  ss << "  (1) The position of the corresponding entity is not specified by Teleport Action";
  throw SemanticError(ss.str());
}

template<typename ... Ts>
decltype(auto) setEntityStatus(Ts && ... xs)
{
  return connection.setEntityStatus(std::forward<decltype(xs)>(xs)...);
}

template<typename ... Ts>
decltype(auto) getCurrentTime(Ts && ... xs)
{
  return connection.getCurrentTime(std::forward<decltype(xs)>(xs)...);
}

template<typename ... Ts>
decltype(auto) isReachedPosition(Ts && ... xs)
{
  return connection.reachPosition(std::forward<decltype(xs)>(xs)...);
}

// template<typename ... Ts>
// decltype(auto) getRelativeDistance(Ts && ... xs)
// {
//   return connection.getRelativeDistance(std::forward<decltype(xs)>(xs)...);
// }

template<typename ... Ts>
decltype(auto) getRelativePose(Ts && ... xs) try {
  return connection.getRelativePose(std::forward<decltype(xs)>(xs)...);
} catch (const simulation_api::SimulationRuntimeError &) {
  geometry_msgs::msg::Pose result {};
  result.position.x = std::numeric_limits<double>::quiet_NaN();
  result.position.y = std::numeric_limits<double>::quiet_NaN();
  result.position.z = std::numeric_limits<double>::quiet_NaN();
  result.orientation.x = 0;
  result.orientation.y = 0;
  result.orientation.z = 0;
  result.orientation.w = 1;
  return result;
}

template<typename ... Ts>
decltype(auto) updateFrame(Ts && ... xs)
{
  return connection.updateFrame(std::forward<decltype(xs)>(xs)...);
}

// template <typename... Ts>
// auto getDistanceAlongRoute(Ts&&... xs)
// {
//   if (const auto result {
//   connection->entity->getLongitudinalDistance(std::forward<decltype(xs)>(xs)...) })
//   {
//     return *result;
//   }
//   else
//   {
//     using value_type = typename std::decay<decltype(result)>::type::value_type;
//     return std::numeric_limits<value_type>::infinity();
//   }
// }

template<typename ... Ts>
auto getTimeHeadway(Ts && ... xs)
{
  const auto result {
    connection.getTimeHeadway(std::forward<decltype(xs)>(xs)...)
  };
  if (result) {
    return result.get();
  } else {
    using value_type = typename std::decay<decltype(result)>::type::value_type;
    return std::numeric_limits<value_type>::quiet_NaN();
  }
}

template<typename ... Ts>
decltype(auto) requestAcquirePosition(Ts && ... xs)
{
  return connection.requestAcquirePosition(std::forward<decltype(xs)>(xs)...);
}

template<typename ... Ts>
auto getStandStillDuration(Ts && ... xs)
{
  const auto result {
    connection.getStandStillDuration(std::forward<decltype(xs)>(xs)...)
  };
  if (result) {
    return result.get();
  } else {
    return static_cast<typename std::decay<decltype(result.get())>::type>(0);
  }
}

template
<
  typename ... Ts
>
decltype(auto) checkCollision(Ts && ... xs)
{
  return connection.checkCollision(std::forward<decltype(xs)>(xs)...);
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__PROCEDURE_HPP_
