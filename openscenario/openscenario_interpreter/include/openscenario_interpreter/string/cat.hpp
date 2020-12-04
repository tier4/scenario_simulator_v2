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

#ifndef OPENSCENARIO_INTERPRETER__STRING__CAT_HPP_
#define OPENSCENARIO_INTERPRETER__STRING__CAT_HPP_

#include <openscenario_interpreter/functional/fold.hpp>

#include <sstream>

namespace openscenario_interpreter
{
inline namespace string
{

auto cat =
  [](auto && ... xs)
  {
    std::stringstream ss {};

    auto write =
      [](auto && os, auto && x)
      {
        os.get() << x;
        return std::forward<decltype(os)>(os);
      };

    fold_left(write, std::ref(ss), std::forward<decltype(xs)>(xs)...);

    return ss.str();
  };

}  // namespace string
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__STRING__CAT_HPP_
