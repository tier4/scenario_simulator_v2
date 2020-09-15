#ifndef SCENARIO_RUNNER__CONCEPTS__OUTPUT_STREAMABLE_HPP_
#define SCENARIO_RUNNER__CONCEPTS__OUTPUT_STREAMABLE_HPP_

#include <iostream>
#include <scenario_runner/type_traits/void_t.hpp>

namespace scenario_runner
{inline namespace concepts
{
template<typename T, typename = void>
struct OutputStreamable
  : public std::false_type
{};

template<typename T>
struct OutputStreamable<T,
  void_t<decltype(std::declval<std::ostream &>() << std::declval<const T &>())>>
  : public std::true_type
{};
}}  // namespace scenario_runner::concepts

#endif  // SCENARIO_RUNNER__CONCEPTS__OUTPUT_STREAMABLE_HPP_
