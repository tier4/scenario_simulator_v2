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

#ifndef OPENSCENARIO_INTERPRETER__PROCEDURE_HPP_
#define OPENSCENARIO_INTERPRETER__PROCEDURE_HPP_

#include <limits>
#include <memory>
#include <openscenario_interpreter/error.hpp>
#include <traffic_simulator/api/api.hpp>
#include <utility>

namespace openscenario_interpreter
{
class SimulatorCore
{
public:  // TODO PRIVATE!
  static inline std::unique_ptr<traffic_simulator::API> connection = nullptr;

public:
  template <typename... Ts>
  static auto activate(Ts &&... xs) -> void
  {
    if (not connection) {
      connection = std::make_unique<traffic_simulator::API>(std::forward<decltype(xs)>(xs)...);
    } else {
      throw Error("The simulator core has already been instantiated.");
    }
  }

  static auto deactivate() -> void { connection.reset(); }

  class GeneralCommand  // OpenSCENARIO 1.1.1 Section 3.1.5
  {
  protected:
    template <typename Metric, typename... Ts>
    static auto addMetric(Ts &&... xs) -> void
    {
      connection->addMetric<Metric>(std::forward<Ts>(xs)...);
    }
  };

  class ActionApplication  // OpenSCENARIO 1.1.1 Section 3.1.5
  {
  protected:
  };

  class ConditionEvaluation  // OpenSCENARIO 1.1.1 Section 3.1.5
  {
    template <typename... Ts>
    auto currentEntityStatus(Ts &&... xs)
    {
      return connection->getEntityStatus(std::forward<decltype(xs)>(xs)...);
    }

  protected:
    template <typename... Ts>
    auto evaluateSpeed(Ts &&... xs)
    {
      return currentEntityStatus(std::forward<decltype(xs)>(xs)...).action_status.twist.linear.x;
    }

    template <typename... Ts>
    auto evaluateAcceleration(Ts &&... xs)
    {
      return currentEntityStatus(std::forward<decltype(xs)>(xs)...).action_status.accel.linear.x;
    }
  };
};

template <typename... Ts>
auto getRelativePose(Ts &&... xs)
try {
  return SimulatorCore::connection->getRelativePose(std::forward<decltype(xs)>(xs)...);
} catch (...) {
  geometry_msgs::msg::Pose result{};
  result.position.x = std::numeric_limits<double>::quiet_NaN();
  result.position.y = std::numeric_limits<double>::quiet_NaN();
  result.position.z = std::numeric_limits<double>::quiet_NaN();
  result.orientation.x = 0;
  result.orientation.y = 0;
  result.orientation.z = 0;
  result.orientation.w = 1;
  return result;
}

auto toLanePosition(const geometry_msgs::msg::Pose & pose) ->
  typename std::decay<decltype(SimulatorCore::connection
                                 ->toLaneletPose(std::declval<decltype(pose)>(), false)
                                 .get())>::type;

#define STRIP_OPTIONAL(IDENTIFIER, ALTERNATE)                                                     \
  template <typename... Ts>                                                                       \
  auto IDENTIFIER(Ts &&... xs)                                                                    \
  {                                                                                               \
    const auto result = SimulatorCore::connection->IDENTIFIER(std::forward<decltype(xs)>(xs)...); \
    if (result) {                                                                                 \
      return result.get();                                                                        \
    } else {                                                                                      \
      using value_type = typename std::decay<decltype(result)>::type::value_type;                 \
      return ALTERNATE;                                                                           \
    }                                                                                             \
  }                                                                                               \
  static_assert(true, "")

STRIP_OPTIONAL(getBoundingBoxDistance, static_cast<value_type>(0));
STRIP_OPTIONAL(getLongitudinalDistance, std::numeric_limits<value_type>::quiet_NaN());
STRIP_OPTIONAL(getStandStillDuration, static_cast<value_type>(0));
STRIP_OPTIONAL(getTimeHeadway, std::numeric_limits<value_type>::quiet_NaN());

#undef STRIP_OPTIONAL

#define FORWARD_TO_SIMULATION_API(IDENTIFIER)                                        \
  template <typename... Ts>                                                          \
  decltype(auto) IDENTIFIER(Ts &&... xs)                                             \
  {                                                                                  \
    return SimulatorCore::connection->IDENTIFIER(std::forward<decltype(xs)>(xs)...); \
  }                                                                                  \
  static_assert(true, "")

FORWARD_TO_SIMULATION_API(attachDetectionSensor);
FORWARD_TO_SIMULATION_API(attachLidarSensor);
FORWARD_TO_SIMULATION_API(engage);
FORWARD_TO_SIMULATION_API(getCurrentAction);
FORWARD_TO_SIMULATION_API(getCurrentTime);
FORWARD_TO_SIMULATION_API(getDriverModel);
FORWARD_TO_SIMULATION_API(getTrafficRelationReferees);
FORWARD_TO_SIMULATION_API(initialize);
FORWARD_TO_SIMULATION_API(isInLanelet);
FORWARD_TO_SIMULATION_API(ready);
FORWARD_TO_SIMULATION_API(requestLaneChange);
FORWARD_TO_SIMULATION_API(requestSpeedChange);
FORWARD_TO_SIMULATION_API(setEntityStatus);
FORWARD_TO_SIMULATION_API(setVelocityLimit);
FORWARD_TO_SIMULATION_API(updateFrame);

#undef FORWARD_TO_SIMULATION_API

#define RENAME(TO, FROM)                                                       \
  template <typename... Ts>                                                    \
  decltype(auto) TO(Ts &&... xs)                                               \
  {                                                                            \
    return SimulatorCore::connection->FROM(std::forward<decltype(xs)>(xs)...); \
  }                                                                            \
  static_assert(true, "")

// NOTE: See OpenSCENARIO 1.1 Figure 2. Actions and conditions

RENAME(applyAcquirePositionAction, requestAcquirePosition);
RENAME(applyAddEntityAction, spawn);
RENAME(applyAssignControllerAction, setDriverModel);
RENAME(applyAssignRouteAction, requestAssignRoute);
RENAME(applyDeleteEntityAction, despawn);
RENAME(applyLaneChangeAction, requestLaneChange);
RENAME(applyTeleportAction, setEntityStatus);
RENAME(applyWalkStraightAction, requestWalkStraight);
RENAME(evaluateCollisionCondition, checkCollision);
RENAME(evaluateCurrentEmergencyState, getEmergencyStateString);
RENAME(evaluateCurrentState, getCurrentAction);
RENAME(evaluateReachPositionCondition, reachPosition);
RENAME(toWorldPosition, toMapPose);

#undef RENAME
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__PROCEDURE_HPP_
