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

#ifndef CONCEALER__IS_PACKAGE_EXISTS_HPP_
#define CONCEALER__IS_PACKAGE_EXISTS_HPP_

namespace concealer
{
auto isPackageExists(const std::string & package_name) noexcept -> bool;
}  // namespace concealer
#endif  // CONCEALER__IS_PACKAGE_EXISTS_HPP_
