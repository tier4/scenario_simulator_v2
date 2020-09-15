#ifndef SCENARIO_RUNNER__CONCEPTS__EVALUABLE_HPP_
#define SCENARIO_RUNNER__CONCEPTS__EVALUABLE_HPP_

#include <scenario_runner/type_traits/void_t.hpp>

namespace scenario_runner
{inline namespace concepts
{
template<typename T, typename = void>
struct Evaluable
  : public std::false_type
{};

template<typename T>
struct Evaluable<T, void_t<decltype(std::declval<T>().evaluate())>>
  : public std::true_type
{};
}}  // namespace scenario_runner::concepts

#endif  // SCENARIO_RUNNER__CONCEPTS__EVALUABLE_HPP_
