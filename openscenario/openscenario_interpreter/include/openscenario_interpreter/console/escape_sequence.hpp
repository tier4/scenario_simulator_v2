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

#ifndef OPENSCENARIO_INTERPRETER__CONSOLE__ESCAPE_SEQUENCE_HPP_
#define OPENSCENARIO_INTERPRETER__CONSOLE__ESCAPE_SEQUENCE_HPP_

#include <openscenario_interpreter/console/is_console.hpp>
#include <string>

namespace openscenario_interpreter
{
inline namespace console
{
template <typename... Ts>
auto & escape_sequence(std::basic_ostream<Ts...> & os, const std::string & code)
{
  return is_console(os) ? (os << "\x1b" << code) : os;
}

#define BOILERPLATE(CODE, NAME) \
  auto NAME = [](std::ostream & os) -> decltype(auto) { return escape_sequence(os, CODE); }

BOILERPLATE("[0m", reset);
BOILERPLATE("[1m", bold);
BOILERPLATE("[2m", faint);
BOILERPLATE("[3m", italic);  // Not widely supported. Sometimes treated as inverse.
BOILERPLATE("[4m", underline);
BOILERPLATE("[5m", slow_blink);   // Less than 150 per minute.
BOILERPLATE("[6m", rapid_blink);  // More than 150 per minute. Not widely supported.
BOILERPLATE("[7m", reverse);
BOILERPLATE("[8m", conceal);  // Not widely supported.

inline namespace foreground
{
BOILERPLATE("[30m", black);
BOILERPLATE("[31m", red);
BOILERPLATE("[32m", green);
BOILERPLATE("[33m", yellow);
BOILERPLATE("[34m", blue);
BOILERPLATE("[35m", magenta);
BOILERPLATE("[36m", cyan);
BOILERPLATE("[37m", white);
}  // namespace foreground

namespace background
{
BOILERPLATE("[40m", black);
BOILERPLATE("[41m", red);
BOILERPLATE("[42m", green);
BOILERPLATE("[43m", yellow);
BOILERPLATE("[44m", blue);
BOILERPLATE("[45m", magenta);
BOILERPLATE("[46m", cyan);
BOILERPLATE("[47m", white);
}  // namespace background

#undef BOILERPLATE
}  // namespace console
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__CONSOLE__ESCAPE_SEQUENCE_HPP_
