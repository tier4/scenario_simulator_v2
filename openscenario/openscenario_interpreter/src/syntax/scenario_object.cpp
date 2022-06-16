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

#include <boost/lexical_cast.hpp>
#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/controller.hpp>
#include <openscenario_interpreter/syntax/scenario_object.hpp>
#include <traffic_simulator/metrics/out_of_range_metric.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
ScenarioObject::ScenarioObject(const pugi::xml_node & node, Scope & scope)
: Scope(readAttribute<String>("name", node, scope), scope),
  EntityObject(node, local()),
  object_controller(readElement<ObjectController>("ObjectController", node, local()))
{
}

auto ScenarioObject::activateOutOfRangeMetric(const Vehicle & vehicle) const -> bool
{
  metrics::OutOfRangeMetric::Config configuration;

  configuration.target_entity = name;
  configuration.min_velocity = -vehicle.performance.max_speed;
  configuration.max_velocity = +vehicle.performance.max_speed;
  configuration.min_acceleration = -vehicle.performance.max_deceleration;
  configuration.max_acceleration = +vehicle.performance.max_acceleration;
  configuration.min_jerk =
    object_controller.is<Controller>()
      ? object_controller.as<Controller>().properties.get<Double>("minJerk", Double::lowest())
      : Double::lowest();
  configuration.max_jerk =
    object_controller.is<Controller>()
      ? object_controller.as<Controller>().properties.get<Double>("maxJerk", Double::max())
      : Double::max();
  configuration.jerk_topic = "/planning/scenario_planning/motion_velocity_optimizer/closest_jerk";

  addMetric<metrics::OutOfRangeMetric>(name + "-out-of-range", configuration);

  return true;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
