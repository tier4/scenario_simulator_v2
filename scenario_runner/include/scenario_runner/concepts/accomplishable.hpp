#ifndef SCENARIO_RUNNER__CONCEPTS__ACCOMPLISHABLE_HPP_
#define SCENARIO_RUNNER__CONCEPTS__ACCOMPLISHABLE_HPP_

#include <scenario_runner/type_traits/void_t.hpp>

namespace scenario_runner
{inline namespace concepts
{
template<typename T, typename = void>
struct Accomplishable
  : public std::false_type
{};

template<typename T>
struct Accomplishable<T, void_t<decltype(std::declval<T>().accomplished())>>
  : public std::true_type
{};
}}  // namespace scenario_runner::concepts

#endif  // SCENARIO_RUNNER__CONCEPTS__ACCOMPLISHABLE_HPP_
