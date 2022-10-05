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

#include <memory>
#include <openscenario_interpreter/utility/demangle.hpp>

#ifdef __GNUC__
#include <cxxabi.h>
#endif

namespace openscenario_interpreter
{
inline namespace utility
{
auto demangle(const char * name) -> std::string
{
#ifdef __GNUC__
  int failed = 0;

  std::unique_ptr<char, decltype(&std::free)> demangled{
    abi::__cxa_demangle(name, nullptr, nullptr, &failed),
    [](void * x) noexcept -> void { std::free(x); }};

  return std::string(failed ? name : demangled.value());
#else
  return std::string(name);
#endif
}

auto demangle(const std::type_info & info) -> std::string { return demangle(info.name()); }
}  // namespace utility
}  // namespace openscenario_interpreter
