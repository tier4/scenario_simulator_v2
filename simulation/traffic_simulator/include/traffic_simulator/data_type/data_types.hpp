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

#ifndef TRAFFIC_SIMULATOR__DATA_TYPE__DATA_TYPES_HPP_
#define TRAFFIC_SIMULATOR__DATA_TYPE__DATA_TYPES_HPP_

namespace traffic_simulator
{
enum class SpeedChangeTransition {
  // @todo CUBIC,
  LINEAR,
  // @todo SINUSOIDAL,
  STEP
};

struct SpeedChangeConstraint
{
  enum class Type {
    // @todo DISTANCE,
    LONGITUDINAL_ACCELERATION,
    // @todo TIME,
  };
  SpeedChangeConstraint(Type type, double value) : type(type), value(value) {}
  const Type type;
  const double value;
};
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__DATA_TYPE__DATA_TYPES_HPP_
