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

#ifndef SIMULATION_INTERFACE__OPERATORS_HPP_
#define SIMULATION_INTERFACE__OPERATORS_HPP_

#include <simulation_interface/traffic_simulator_msgs.pb.h>

#include <ostream>
#include <traffic_simulator_msgs/msg/entity_subtype.hpp>
#include <traffic_simulator_msgs/msg/entity_type.hpp>

namespace traffic_simulator_msgs
{
std::ostream & operator<<(std::ostream &, const traffic_simulator_msgs::msg::EntityType &);

std::ostream & operator<<(std::ostream &, const traffic_simulator_msgs::EntityType &);

std::ostream & operator<<(std::ostream &, const traffic_simulator_msgs::msg::EntitySubtype &);

std::ostream & operator<<(std::ostream &, const traffic_simulator_msgs::EntitySubtype &);
}  // namespace traffic_simulator_msgs

#endif  // SIMULATION_INTERFACE__OPERATORS_HPP_
