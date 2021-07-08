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

#include <memory>
#include <openscenario_interpreter/procedure.hpp>

namespace openscenario_interpreter
{
static typename std::aligned_storage<
  sizeof(traffic_simulator::API), alignof(traffic_simulator::API)>::type memory;

traffic_simulator::API & connection = reinterpret_cast<traffic_simulator::API &>(memory);

auto toLanePosition(const geometry_msgs::msg::Pose & pose) -> typename std::decay<
  decltype(connection.toLaneletPose(std::declval<decltype(pose)>()).get())>::type
{
  const auto result = connection.toLaneletPose(pose);

  if (result) {
    return result.get();
  } else {
    // clang-format off
    throw SimulationError(
      "The specified WorldPosition = [", pose.position.x, ", ",
                                         pose.position.y, ", ",
                                         pose.position.z, "] could not be "
      "approximated to the proper Lane. Perhaps the WorldPosition points to a "
      "location where multiple lanes overlap, and there are at least two or "
      "more candidates for a LanePosition that can be approximated to that "
      "WorldPosition. This issue can be resolved by strictly specifying the "
      "location using LanePosition instead of WorldPosition");
    // clang-format on
  }
}
}  // namespace openscenario_interpreter
