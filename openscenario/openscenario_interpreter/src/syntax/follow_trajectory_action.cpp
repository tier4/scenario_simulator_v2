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

auto FollowTrajectoryAction::accomplished() noexcept -> bool { return false; }

auto FollowTrajectoryAction::endsImmediately() noexcept -> bool { return false; }

auto FollowTrajectoryAction::run() -> void { LINE(); }

auto FollowTrajectoryAction::start() -> void
{
  for (auto && actor : actors) {
    if (trajectory_ref.trajectory.as<Trajectory>().shape.is<Polyline>()) {
      auto polyline = [this]() {
        auto absolute = [this](auto && time) -> std::optional<double> {
          if (time_reference.is<Timing>()) {
            switch (time_reference.as<Timing>().domain_absolute_relative) {
              case ReferenceContext::relative:
                return evaluateSimulationTime() + time_reference.as<Timing>().offset + time;
              default:
              case ReferenceContext::absolute:
                return time_reference.as<Timing>().offset + time;
            }
          } else {
            return std::nullopt;
          }
        };

        auto && polyline = traffic_simulator::follow_trajectory::Polyline();

        for (auto && vertex :
             trajectory_ref.trajectory.as<Trajectory>().shape.as<Polyline>().vertices) {
          polyline.vertices.push_back(traffic_simulator::follow_trajectory::Vertex(
            absolute(vertex.time), static_cast<geometry_msgs::msg::Pose>(vertex.position)));
        }

        return polyline;
      };

      applyFollowTrajectoryAction(
        actor, traffic_simulator::follow_trajectory::Parameter(
                 initial_distance_offset,
                 trajectory_following_mode.following_mode == FollowingMode::position,
                 trajectory_ref.trajectory.as<Trajectory>().closed, polyline()));
    }
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
