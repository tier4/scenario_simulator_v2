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
    configuration.min_velocity = -parameters.performance.max_speed;
    configuration.max_velocity = +parameters.performance.max_speed;
    configuration.min_acceleration = -parameters.performance.max_deceleration;
    configuration.max_acceleration = +parameters.performance.max_acceleration;

    if (object_controller.is<Controller>()) {
      auto controller = object_controller.as<Controller>();
      auto max_jerk = controller["maxJerk"];
      auto min_jerk = controller["minJerk"];

      if (not max_jerk.name.empty()) {
        configuration.max_jerk = boost::lexical_cast<double>(max_jerk.value);
      }

      if (not min_jerk.name.empty()) {
        configuration.min_jerk = boost::lexical_cast<double>(min_jerk.value);
      }
    }

    if (object_controller.isUserDefinedController()) {
      configuration.jerk_topic =
        "/planning/scenario_planning/motion_velocity_optimizer/closest_jerk";
    }
  }

  connection.addMetric<metrics::OutOfRangeMetric>(name + "-out-of-range", configuration);

  return true;
}

auto ScenarioObject::activateSensors() -> bool
{
  if (object_controller.isUserDefinedController()) {
    const auto architecture_type = getParameter<std::string>("architecture_type", "");
    if (architecture_type == "tier4/proposal") {
      return attachLidarSensor(traffic_simulator::helper::constructLidarConfiguration(
               traffic_simulator::helper::LidarType::VLP16, name,
               "/sensing/lidar/no_ground/pointcloud")) and
             attachDetectionSensor(traffic_simulator::helper::constructDetectionSensorConfiguration(
               name, "/perception/object_recognition/objects", 0.1));
    } else if (architecture_type == "awf/auto") {
      /*
         Autoware.Auto does not currently support object prediction however it
         is work-in-progress for Cargo ODD msgs are already implemented and
         autoware_auto_msgs::msg::PredictedObjects will probably be used here
         topic name is yet unknown.
      */
      return attachLidarSensor(traffic_simulator::helper::constructLidarConfiguration(
        traffic_simulator::helper::LidarType::VLP16, name, "/perception/points_nonground"));
    } else {
      throw SemanticError(
        "Unexpected architecture_type ", std::quoted(architecture_type), " specified");
    }
  } else {
    return true;
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
