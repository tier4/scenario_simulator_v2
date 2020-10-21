// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef OPEN_SCENARIO_INTERPRETER__ACCESSOR_HPP_
#define OPEN_SCENARIO_INTERPRETER__ACCESSOR_HPP_

#include <simulation_api/api/api.hpp>
#include <open_scenario_interpreter/error.hpp>

#include <limits>
#include <memory>
#include <utility>

namespace open_scenario_interpreter
{
class Accessor
{
  static std::unique_ptr<scenario_simulator::API> connection;

  static const auto & access()
  {
    if (connection) {
      return *connection;
    } else {
      throw std::runtime_error("connection-error");
    }
  }

public:
  static auto ready() noexcept
  {
    return static_cast<bool>(connection);
  }

  template<typename ... Ts>
  static decltype(auto) connect(Ts && ... xs)
  {
    connection.reset(new scenario_simulator::API(std::forward<decltype(xs)>(xs)...));
    return ready();
  }

  template<typename ... Ts>
  static decltype(auto) initialize(Ts && ... xs)
  {
    return access().simulation->initialize(std::forward<decltype(xs)>(xs)...);
  }

protected:
  template<typename ... Ts>
  decltype(auto) spawn(Ts && ... xs) const
  {
    return access().entity->spawn(std::forward<decltype(xs)>(xs)...);
  }

  template<typename ... Ts>
  decltype(auto) requestLaneChange(Ts && ... xs) const
  {
    return access().entity->requestLaneChange(std::forward<decltype(xs)>(xs)...);
  }

  template<typename ... Ts>
  decltype(auto) isInLanelet(Ts && ... xs) const
  {
    return access().entity->isInLanelet(std::forward<decltype(xs)>(xs)...);
  }

  template<typename ... Ts>
  decltype(auto) setTargetSpeed(Ts && ... xs) const
  {
    return access().entity->setTargetSpeed(std::forward<decltype(xs)>(xs)...);
  }

  template<typename ... Ts>
  decltype(auto) getEntityStatus(Ts && ... xs) const try {
    return access().entity->getEntityStatus(std::forward<decltype(xs)>(xs)...);
  } catch (const simulation_api::SimulationRuntimeError & error) {
    std::stringstream ss {};
    ss << error.what() << ".\n";
    ss << "Possible causes:\n";
    ss << "  (1) The position of the corresponding entity is not specified by Teleport Action";
    throw SemanticError(ss.str());
  }

  template<typename ... Ts>
  decltype(auto) setEntityStatus(Ts && ... xs) const
  {
    return access().entity->setEntityStatus(std::forward<decltype(xs)>(xs)...);
  }

  template<typename ... Ts>
  decltype(auto) getCurrentTime(Ts && ... xs) const
  {
    return access().simulation->getCurrentTime(std::forward<decltype(xs)>(xs)...);
  }

  template<typename ... Ts>
  decltype(auto) isReachedPosition(Ts && ... xs) const
  {
    return access().entity->reachPosition(std::forward<decltype(xs)>(xs)...);
  }

  // template<typename ... Ts>
  // decltype(auto) getRelativeDistance(Ts && ... xs) const
  // {
  //   return access().entity->getRelativeDistance(std::forward<decltype(xs)>(xs)...);
  // }

  template<typename ... Ts>
  decltype(auto) getRelativePose(Ts && ... xs) const
  {
    return access().entity->getRelativePose(std::forward<decltype(xs)>(xs)...);
  }

  template<typename ... Ts>
  decltype(auto) updateFrame(Ts && ... xs) const
  {
    return access().simulation->updateFrame(std::forward<decltype(xs)>(xs)...);
  }

  // template <typename... Ts>
  // auto getDistanceAlongRoute(Ts&&... xs) const
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
  auto getTimeHeadway(Ts && ... xs) const
  {
    const auto result {
      access().entity->getTimeHeadway(std::forward<decltype(xs)>(xs)...)
    };
    if (result) {
      return *result;
    } else {
      using value_type = typename std::decay<decltype(result)>::type::value_type;
      return std::numeric_limits<value_type>::quiet_NaN();
    }
  }

  template<typename ... Ts>
  decltype(auto) requestAcquirePosition(Ts && ... xs) const
  {
    return access().entity->requestAcquirePosition(std::forward<decltype(xs)>(xs)...);
  }
};
}  // namespace open_scenario_interpreter

#endif  // OPEN_SCENARIO_INTERPRETER__ACCESSOR_HPP_
