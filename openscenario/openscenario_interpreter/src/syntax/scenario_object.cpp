// Copyright 2015-2021 Tier IV, Inc. All rights reserved.
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
  {
    const auto parameters = static_cast<traffic_simulator_msgs::msg::VehicleParameters>(vehicle);

    configuration.target_entity = name;
    configuration.min_velocity = -parameters.performance.vel_lim;
    configuration.max_velocity = +parameters.performance.vel_lim;
    configuration.min_acceleration = -parameters.performance.max_deceleration;
    configuration.max_acceleration = +parameters.performance.vel_rate_lim;

    if (object_controller.is<Controller>()) {
      configuration.max_jerk = object_controller.as<Controller>().properties.get<Double>(
        "maxJerk", std::numeric_limits<Double::value_type>::max());
      configuration.min_jerk = object_controller.as<Controller>().properties.get<Double>(
        "minJerk", std::numeric_limits<Double::value_type>::lowest());
    }

    if (object_controller.isUserDefinedController()) {
      configuration.jerk_topic =
        "/planning/scenario_planning/motion_velocity_optimizer/closest_jerk";
    }
  }

  addMetric<metrics::OutOfRangeMetric>(name + "-out-of-range", configuration);

  return true;
}

auto ScenarioObject::activateSensors() -> bool
{
  /*
     NOTE: The term "controller" in OpenSCENARIO is a concept equivalent to
     "the person driving the car. Here, Autoware is considered anthropomorphic.
     In other words, the sensor performance of Autoware in a simulation is
     described in ScenarioObject.ObjectController.Controller.Properties as
     "characteristics of the person driving the car.
  */
  simulation_api_schema::DetectionSensorConfiguration configuration;
  {
    configuration.set_entity(name);
    configuration.set_architecture_type(
      getParameter<std::string>("architecture_type", "awf/universe"));
    configuration.set_update_duration(0.1);
    configuration.set_range(300);
    configuration.set_filter_by_range(
      object_controller.is<Controller>()
        ? object_controller.as<Controller>().properties.get<Boolean>("isClairvoyant")
        : false);
  }

  return object_controller.isUserDefinedController() and attachLidarSensor(name) and
         attachDetectionSensor(configuration);
}
}  // namespace syntax
}  // namespace openscenario_interpreter
