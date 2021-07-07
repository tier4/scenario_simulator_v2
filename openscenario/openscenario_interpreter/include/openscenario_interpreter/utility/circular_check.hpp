
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

#ifndef OPENSCENARIO_INTERPRETER__UTILITY__CIRCULAR_CHECK_HPP_
#define OPENSCENARIO_INTERPRETER__UTILITY__CIRCULAR_CHECK_HPP_

#include <unordered_map>
#include <utility>

namespace openscenario_interpreter
{
inline namespace utility
{
namespace detail
{
template <class Node, class NodeToChildren, class NodeToBool>
bool circular_check(const Node & node, NodeToChildren && node_to_children, NodeToBool & visit_flag)
{
  if (visit_flag[node]) {
    return true;
  }

  visit_flag[node] = true;

  for (auto && child : node_to_children(node)) {
    if (child == node) continue;

    if (circular_check(child, node_to_children, visit_flag)) {
      return true;
    }
    visit_flag[child] = false;
  }

  return false;
}
}  // namespace detail

template <class Node, class NodeToChildren, class Hash = std::hash<Node>>
bool circular_check(const Node & init, NodeToChildren && node_to_children)
{
  std::unordered_map<Node, bool, Hash> visit_flag;
  return detail::circular_check(init, node_to_children, visit_flag);
}
}  // namespace utility
}  // namespace openscenario_interpreter
#endif  // OPENSCENARIO_INTERPRETER__UTILITY__CIRCULAR_CHECK_HPP_
