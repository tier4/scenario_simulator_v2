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

#ifndef SCENARIO_SIMULATOR_EXCEPTION__EXCEPTION_HPP_
#define SCENARIO_SIMULATOR_EXCEPTION__EXCEPTION_HPP_

#include <scenario_simulator_exception/concatenate.hpp>
#include <stdexcept>
#include <utility>

namespace common
{
inline namespace scenario_simulator_exception
{
struct Error : public std::runtime_error
{
  template <typename... Ts>
  explicit Error(Ts &&... xs)
  : std::runtime_error{concatenate(std::forward<decltype(xs)>(xs)..., ".")}
  {
  }
};

#define DEFINE_ERROR_CATEGORY(TYPENAME)                                                       \
  struct TYPENAME : public Error                                                              \
  {                                                                                           \
    template <typename... Ts>                                                                 \
    explicit TYPENAME(Ts &&... xs) : Error{#TYPENAME ": ", std::forward<decltype(xs)>(xs)...} \
    {                                                                                         \
    }                                                                                         \
  }

#define THROW_ERROR(TYPENAME, ...) throw TYPENAME(__FILE__, ":", __LINE__, ": ", __VA_ARGS__);

// Autoware encountered some problem that led to a simulation failure.
DEFINE_ERROR_CATEGORY(AutowareError);
#define THROW_AUTOWARE_ERROR(...) \
  THROW_ERROR(common::scenario_simulator_exception::AutowareError, __VA_ARGS__);

// Although there were no syntactic errors in the description of the scenario,
// differences in meaning and inconsistencies were found.
DEFINE_ERROR_CATEGORY(SemanticError);
#define THROW_SEMANTIC_ERROR(...) \
  THROW_ERROR(common::scenario_simulator_exception::SemanticError, __VA_ARGS__);

// A problem occurred that interfered with the continuation of the simulation.
DEFINE_ERROR_CATEGORY(SimulationError);
#define THROW_SIMULATION_ERROR(...) \
  THROW_ERROR(common::scenario_simulator_exception::SimulationError, __VA_ARGS__);

// There is a syntactic error in the description of the scenario. Or you are
// using a feature that is not yet supported by our implementation.
DEFINE_ERROR_CATEGORY(SyntaxError);
#define THROW_SYNTAX_ERROR(...) \
  THROW_ERROR(common::scenario_simulator_exception::SyntaxError, __VA_ARGS__);

DEFINE_ERROR_CATEGORY(SpecificationViolationError);
#define THROW_SPECIFICATION_VIOLATION_ERROR(...) \
  THROW_ERROR(common::scenario_simulator_exception::SpecificationViolationError, __VA_ARGS__);

#define SPECIFICATION_VIOLATION_ERROR(...) \
  common::scenario_simulator_exception::SpecificationViolationError(__FILE__, ":", __LINE__, ": ", __VA_ARGS__);

#undef DEFINE_ERROR_CATEGORY

}  // namespace scenario_simulator_exception
}  // namespace common

#endif  // SCENARIO_SIMULATOR_EXCEPTION__EXCEPTION_HPP_
