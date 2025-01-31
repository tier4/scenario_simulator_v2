// Copyright 2021 Tier IV, Inc All rights reserved.
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

#ifndef CONTEXT_GAMMA_PLANNER__PEDESTRIAN__CONSTRAINTS__CONSTRAINT_ACTIVATOR_HPP_
#define CONTEXT_GAMMA_PLANNER__PEDESTRIAN__CONSTRAINTS__CONSTRAINT_ACTIVATOR_HPP_

#include <customized_rvo2/RVO.h>

#include <context_gamma_planner/constraints/constraint_activator_base.hpp>
#include <geometry/spline/catmull_rom_subspline.hpp>
#include <memory>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/traffic_lights/traffic_lights.hpp>

namespace context_gamma_planner
{
namespace pedestrian
{
namespace constraints
{
class ConstraintActivator : public context_gamma_planner::constraints::ConstraintActivatorBase
{
public:
  ConstraintActivator(
    const std::shared_ptr<hdmap_utils::HdMapUtils> hd_map_utils_ptr,
    const std::shared_ptr<traffic_simulator::TrafficLightsBase> traffic_lights_ptr);
};
}  // namespace constraints
}  // namespace pedestrian
}  // namespace context_gamma_planner

#endif  // CONTEXT_GAMMA_PLANNER__PEDESTRIAN__CONSTRAINTS__CONSTRAINT_ACTIVATOR_HPP_
