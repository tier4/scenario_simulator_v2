#ifndef SCENARIO_RUNNER__ITERATOR__SIZE_HPP_
#define SCENARIO_RUNNER__ITERATOR__SIZE_HPP_

#include <iterator>

namespace scenario_runner
{inline namespace iterator
{
template<typename T>
auto size(const T & range)
{
  return std::distance(std::begin(range), std::end(range));
}
}}  // namespace scenario_runner::iterator

#endif  // SCENARIO_RUNNER__ITERATOR__SIZE_HPP_
