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

#include <openscenario_interpreter/functional/equal_to.hpp>
#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/speed_profile_action.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
SpeedProfileAction::SpeedProfileAction(const pugi::xml_node & node, Scope & scope)
: Scope(scope),
  entity_ref(readAttribute<EntityRef>("entityRef", node, scope, "")),
  following_mode(readAttribute<FollowingMode>("followingMode", node, scope)),
  dynamic_constraints(
    readElement<DynamicConstraints>("DynamicConstraints", node, scope, DynamicConstraints())),
  speed_profile_entry(readElements<SpeedProfileEntry, 1>("SpeedProfileEntry", node, scope))
{
}

auto SpeedProfileAction::apply(
  const EntityRef & actor, const SpeedProfileEntry & speed_profile_entry) -> void
{
  auto absolute_target_speed = [&]() { return speed_profile_entry.speed; };

  auto relative_target_speed = [&]() {
    return traffic_simulator::speed_change::RelativeTargetSpeed(
      entity_ref,                                                         //
      traffic_simulator::speed_change::RelativeTargetSpeed::Type::DELTA,  //
      speed_profile_entry.speed);
  };

  auto constraint = [&]() {
    if (std::isnan(speed_profile_entry.time)) {
      switch (following_mode) {
        case FollowingMode::position:
          return traffic_simulator::speed_change::Constraint(
            traffic_simulator::speed_change::Constraint::Type::LONGITUDINAL_ACCELERATION,
            traffic_simulator_msgs::msg::BehaviorParameter().dynamic_constraints.max_acceleration);
        default:
        case FollowingMode::follow:
          return traffic_simulator::speed_change::Constraint(
            traffic_simulator::speed_change::Constraint::Type::NONE, 0);
      }
    } else {
      return traffic_simulator::speed_change::Constraint(
        traffic_simulator::speed_change::Constraint::Type::TIME, speed_profile_entry.time);
    }
  };

  auto transition = [&]() {
    switch (following_mode) {
      case FollowingMode::position:
        return traffic_simulator::speed_change::Transition::LINEAR;
      default:
      case FollowingMode::follow:
        return traffic_simulator::speed_change::Transition::AUTO;
    }
  };

  std::cout << "BEFORE APPLY PROFILE!!!" << std::endl;
  PRINT(getBehaviorParameter(actor).dynamic_constraints.max_speed);
  PRINT(getBehaviorParameter(actor).dynamic_constraints.max_acceleration);
  PRINT(getBehaviorParameter(actor).dynamic_constraints.max_acceleration_rate);
  PRINT(getBehaviorParameter(actor).dynamic_constraints.max_deceleration);
  PRINT(getBehaviorParameter(actor).dynamic_constraints.max_deceleration_rate);

  applyProfileAction(actor, dynamic_constraints);

  std::cout << "AFTER APPLY PROFILE!!!" << std::endl;
  PRINT(getBehaviorParameter(actor).dynamic_constraints.max_speed);
  PRINT(getBehaviorParameter(actor).dynamic_constraints.max_acceleration);
  PRINT(getBehaviorParameter(actor).dynamic_constraints.max_acceleration_rate);
  PRINT(getBehaviorParameter(actor).dynamic_constraints.max_deceleration);
  PRINT(getBehaviorParameter(actor).dynamic_constraints.max_deceleration_rate);

  if (entity_ref.empty()) {
    applySpeedAction(
      actor, absolute_target_speed(), transition(), constraint(),
      std::isnan(speed_profile_entry.time));
  } else {
    applySpeedAction(
      actor, relative_target_speed(), transition(), constraint(),
      std::isnan(speed_profile_entry.time));
  }
}

auto SpeedProfileAction::accomplished() -> bool
{
  // NOTE: Action ends on reaching the target speed of the last SpeedProfileEntry.
  return std::all_of(
    std::begin(accomplishments), std::end(accomplishments), [this](auto && actor_and_iter) {
      return std::get<1>(actor_and_iter) == std::end(speed_profile_entry);
    });
}

auto SpeedProfileAction::endsImmediately() const -> bool { return false; }

auto SpeedProfileAction::run() -> void
{
  auto accomplished = [this](const auto & actor, const auto & speed_profile_entry) {
    if (entity_ref.empty()) {
      return equal_to<double>()(evaluateSpeed(actor), speed_profile_entry.speed);
    } else {
      return equal_to<double>()(
        evaluateSpeed(actor), speed_profile_entry.speed + evaluateSpeed(entity_ref));
    }
  };

  for (auto && [actor, iter] : accomplishments) {
    if (
      iter != std::end(speed_profile_entry) and accomplished(actor, *iter) and
      ++iter != std::end(speed_profile_entry)) {
      apply(actor, *iter);
    }
  }

  for (auto && [actor, iter] : accomplishments) {
    auto target_speed = [&]() -> double {
      if (iter == std::end(speed_profile_entry)) {
        return Double::nan();
      } else if (entity_ref.empty()) {
        return iter->speed;
      } else {
        return evaluateSpeed(entity_ref) + iter->speed;
      }
    };

    auto time = [&]() -> double {
      if (iter == std::end(speed_profile_entry)) {
        return Double::nan();
      } else {
        return iter->time;
      }
    };

    auto previous_time = previous.at(actor).time;
    auto previous_speed = previous.at(actor).speed;
    auto previous_acceleration = previous.at(actor).acceleration;
    auto previous_acceleration_rate = previous.at(actor).acceleration_rate;
    // PRINT(previous_time);
    // PRINT(previous_speed);
    // PRINT(previous_acceleration);
    // PRINT(previous_acceleration_rate);

    auto current_time = evaluateSimulationTime();
    auto current_speed = evaluateSpeed(actor);
    auto current_acceleration = (current_speed - previous_speed) / (current_time - previous_time);
    auto current_acceleration_rate =
      (current_acceleration - previous_acceleration) / (current_time - previous_time);
    // PRINT(current_time);
    // PRINT(current_speed);
    // PRINT(current_acceleration);
    // PRINT(current_acceleration_rate);

    previous.at(actor).time = current_time;
    previous.at(actor).speed = current_speed;
    previous.at(actor).acceleration = current_acceleration;
    previous.at(actor).acceleration_rate = current_acceleration_rate;

    // clang-format off
    std::cout << "actor " << std::quoted(actor) << "\n"
              << "  speed_profile_entry[" << std::distance(std::begin(speed_profile_entry), iter) << "/" << speed_profile_entry.size() - 1 << "]\n"
              << "  current speed = " << current_speed << "\n"
              << "  current acceleration = " << current_acceleration << "\n"
              << "  current acceleration rate = " << current_acceleration_rate << "\n"
              << "  target speed = " << target_speed() << "\n"
              << "  time = " << time() << "\n"
              << "  getBehaviorParameter().max_speed             = " << getBehaviorParameter(actor).dynamic_constraints.max_speed             << "\n"
              << "                        .max_acceleration      = " << getBehaviorParameter(actor).dynamic_constraints.max_acceleration      << "\n"
              << "                        .max_acceleration_rate = " << getBehaviorParameter(actor).dynamic_constraints.max_acceleration_rate << "\n"
              << "                        .max_deceleration      = " << getBehaviorParameter(actor).dynamic_constraints.max_deceleration      << "\n"
              << "                        .max_deceleration_rate = " << getBehaviorParameter(actor).dynamic_constraints.max_deceleration_rate << "\n"
              << std::flush;
    // clang-format on
  }
}

auto SpeedProfileAction::start() -> void
{
  accomplishments.clear();

  previous.clear();

  assert(not speed_profile_entry.empty());

  for (const auto & actor : actors) {
    accomplishments.emplace(actor, std::begin(speed_profile_entry));
    previous.emplace(actor, dynamics());
    apply(actor, *accomplishments[actor]);
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
