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

#include <lanelet2_projection/UTM.h>

#include <filesystem>

namespace map_fragment
{
auto directory() -> const auto &
{
  static const auto directory = std::filesystem::path("/tmp/map_fragment");
  return directory;
}

auto origin() -> const auto &
{
  static const auto origin = lanelet::Origin({35.624285, 139.742570});
  return origin;
}

auto projector() -> const auto &
{
  static const auto projector = lanelet::projection::UtmProjector(origin());
  return projector;
}
}  // namespace map_fragment
