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

#include <openscenario_interpreter/syntax/pedestrian.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
Pedestrian::operator openscenario_msgs::msg::PedestrianParameters() const
{
  openscenario_msgs::msg::PedestrianParameters parameter;
  {
    parameter.name = name;
    parameter.pedestrian_category = boost::lexical_cast<String>(pedestrian_category);
    parameter.bounding_box = static_cast<openscenario_msgs::msg::BoundingBox>(bounding_box);
  }

  return parameter;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
