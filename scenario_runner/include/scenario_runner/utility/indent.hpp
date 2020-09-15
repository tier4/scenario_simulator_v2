#ifndef SCENARIO_RUNNER__UTILITY__INDENT_HPP_
#define SCENARIO_RUNNER__UTILITY__INDENT_HPP_

#include <iostream>

namespace scenario_runner
{inline namespace utility
{
struct Indent
{
  std::size_t depth {0};

  std::size_t width {2};

  auto & reset()
  {
    depth = 0;
    return *this;
  }

  auto & operator++()
  {
    ++depth;
    return *this;
  }

  auto & operator--()
  {
    depth && --depth;
    return *this;
  }

  auto operator++(int)
  {
    Indent result {*this};
    ++depth;
    return result;
  }

  auto operator--(int)
  {
    Indent result {*this};
    depth && --depth;
    return result;
  }
} static indent;

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(std::basic_ostream<Ts...> & os, const Indent & indent)
{
  return os << std::string(indent.depth * 2, ' ');
}
}}  // namespace scenario_runner::utility

#endif  // SCENARIO_RUNNER__UTILITY__INDENT_HPP_
