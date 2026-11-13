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

#ifndef OPENSCENARIO_INTERPRETER__READER__EVALUATE_HPP_
#define OPENSCENARIO_INTERPRETER__READER__EVALUATE_HPP_

#include <iomanip>
#include <openscenario_interpreter/object.hpp>
#include <type_traits>

#include "openscenario_interpreter/utility/demangle.hpp"
#include "scenario_simulator_exception/exception.hpp"

namespace openscenario_interpreter
{
struct Scope;

inline namespace reader
{
std::string evaluate(const std::string &, const Scope &);
}  // namespace reader
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__READER__EVALUATE_HPP_
