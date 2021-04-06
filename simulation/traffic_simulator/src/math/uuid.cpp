/**
 * @file uuid.cpp
 * @author Masaya Kataoka (masaya.kataoka@tier4.jp)
 * @brief source files for generating UUID
 * @version 0.1
 * @date 2021-04-01
 *
 * @copyright Copyright(c) Tier IV.Inc {2015-2021}
 *
 */

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

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/lexical_cast.hpp>

#include <string>

namespace traffic_simulator
{
namespace math
{
std::string generateUUID(const std::string & seed)
{
  boost::uuids::uuid base = boost::uuids::string_generator()("0123456789abcdef0123456789abcdef");
  boost::uuids::name_generator gen(base);
  boost::uuids::uuid uuid = gen(seed);
  return boost::lexical_cast<std::string>(uuid);
}
}  // namespace math
}  // namespace traffic_simulator
