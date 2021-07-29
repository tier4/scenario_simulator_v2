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

#ifndef OPENSCENARIO_INTERPRETER__ITERATOR__CIRCULAR_ITERATOR_HPP_
#define OPENSCENARIO_INTERPRETER__ITERATOR__CIRCULAR_ITERATOR_HPP_

#include <iterator>
#include <type_traits>

namespace openscenario_interpreter
{
inline namespace iterator
{
template <typename Container>
class CircularIterator
{
  using ForwardIterator = typename Container::iterator;
  using ForwardConstIterator = typename Container::const_iterator;

  ForwardIterator begin, end, current;

public:
  using iterator_category = std::forward_iterator_tag;

  using value_type = typename std::iterator_traits<ForwardIterator>::value_type;

  using reference = typename std::add_lvalue_reference<value_type>::type;

  using const_reference = typename std::add_const<reference>::type;

  using pointer = typename std::iterator_traits<ForwardIterator>::pointer;

  using difference_type = typename std::iterator_traits<ForwardIterator>::difference_type;

  CircularIterator() = default;

  explicit CircularIterator(ForwardIterator begin, ForwardIterator end, ForwardIterator current)
  : begin(begin), end(end), current(current)
  {
  }

  CircularIterator & operator=(const CircularIterator &) = default;

  CircularIterator & operator=(const ForwardIterator & iterator)
  {
    current = iterator;
    return *this;
  }

  operator ForwardConstIterator() const { return current; }
  operator ForwardIterator() { return current; }

  reference operator*() const { return *current; }

  auto & operator++()
  {
    if (current == end) {
      current = begin;
    } else if (++current == end) {
      current = begin;
    }
    return *this;
  }

  auto operator++(int)
  {
    auto copy = *this;
    operator++();
    return copy;
  }
};
}  // namespace iterator
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__ITERATOR__CIRCULAR_ITERATOR_HPP_
