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

#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/follow_trajectory_action.hpp>
#include <openscenario_interpreter/syntax/polyline.hpp>
#include <openscenario_interpreter/syntax/timing.hpp>
#include <openscenario_interpreter/syntax/trajectory.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
FollowTrajectoryAction::FollowTrajectoryAction(const pugi::xml_node & node, Scope & scope)
: Scope(scope),
  initial_distance_offset(readAttribute<Double>("initialDistanceOffset", node, local())),
  time_reference(readElement<TimeReference>("TimeReference", node, local())),
  trajectory_following_mode(
    readElement<TrajectoryFollowingMode>("TrajectoryFollowingMode", node, local())),
  trajectory_ref(readElement<TrajectoryRef>("TrajectoryRef", node, local()))
{
}

auto FollowTrajectoryAction::accomplished() -> bool
{
  if (trajectory_ref.trajectory.as<Trajectory>().closed) {
    return false;
  } else if (accomplishments.empty()) {
    /*
       In order to reflect the execution request of FollowTrajectoryAction to
       traffic_simulator, it is necessary to call updateFrame once after
       calling applyFollowTrajectoryAction. This means that evaluating
       accomplished immediately after start will have unexpected results (often
       evaluateCurrentState == "follow_polyline_trajectory" will be false). So
       the first accomplished after calling start will always evaluate to
       false.
    */
    for (const auto & actor : actors) {
      accomplishments.emplace(actor, false);
    }
    return false;
  } else {
    return std::all_of(
      std::begin(accomplishments), std::end(accomplishments), [this](auto && accomplishment) {
        auto is_running = [this](const auto & actor) {
          if (trajectory_ref.trajectory.as<Trajectory>().shape.is<Polyline>()) {
            auto evaluation = actor.apply([&](const auto & object) {
              return evaluateCurrentState(object) == "follow_polyline_trajectory";
            });
            return not evaluation.size() or evaluation.min();
          } else {
            return false;
          }
        };
        return accomplishment.second or
               (accomplishment.second = not is_running(accomplishment.first));
      });
  }
}

auto FollowTrajectoryAction::endsImmediately() noexcept -> bool { return false; }

auto FollowTrajectoryAction::run() -> void {}

auto FollowTrajectoryAction::start() -> void
{
  accomplishments.clear();

  for (const auto & actor : actors) {
    auto repack_trajectory = [this]() {
      if (trajectory_ref.trajectory.as<Trajectory>().shape.is<Polyline>()) {
        auto polyline = traffic_simulator_msgs::msg::Polyline();
        for (auto && vertex :
             trajectory_ref.trajectory.as<Trajectory>().shape.as<Polyline>().vertices) {
          auto message = traffic_simulator_msgs::msg::Vertex();
          /*
             If Vertex.time is unspecified, nan is set as the default value
             (see the openscenario_interpreter::syntax::Vertex constructor).
             This was deliberately chosen because of the convenience of nan's
             behavior of always returning false for any comparison and
             propagating the value even if a quadratic operation is performed.
          */
          message.time =
            time_reference.as<Timing>().offset + time_reference.as<Timing>().scale * vertex.time;
          message.position = static_cast<geometry_msgs::msg::Pose>(vertex.position);
          polyline.vertices.push_back(message);
        }
        return polyline;
      } else {
        throw common::SyntaxError("Non-Polyline trajectory is not yet implemented.");
      }
    };

    auto parameter = std::make_shared<traffic_simulator_msgs::msg::PolylineTrajectory>();

    parameter->initial_distance_offset = initial_distance_offset;
    parameter->dynamic_constraints_ignorable =
      trajectory_following_mode.following_mode == FollowingMode::position;
    parameter->base_time =
      time_reference.as<Timing>().domain_absolute_relative == ReferenceContext::absolute
        ? std::numeric_limits<double>::quiet_NaN()
        : evaluateSimulationTime();
    parameter->closed = trajectory_ref.trajectory.as<Trajectory>().closed;
    parameter->shape = repack_trajectory();

    actor.apply([&](const auto & object) { applyFollowTrajectoryAction(object, parameter); });
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
