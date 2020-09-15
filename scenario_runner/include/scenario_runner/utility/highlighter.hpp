#ifndef SCENARIO_RUNNER__UTILITY__HIGHLIGHTER_HPP_
#define SCENARIO_RUNNER__UTILITY__HIGHLIGHTER_HPP_

#include <iostream>

namespace scenario_runner { inline namespace utility
{
  struct AttributeHighlighter
  {
    const std::string name, value;

    template <typename... Ts>
    decltype(auto) operator ()(std::basic_ostream<Ts...>& os) const
    {
      return os << yellow << name << reset << "=" << cyan << "\"" << value << "\"" << reset;
    }
  };

  template <typename... Ts>
  decltype(auto) operator <<(std::basic_ostream<Ts...>& os, const AttributeHighlighter& highlight)
  {
    return highlight(os);
  }

  template <typename T,
            typename = typename std::enable_if<OutputStreamable<T>::value>::type>
  auto highlight(const std::string& name, const T& value)
  {
    return AttributeHighlighter { name, boost::lexical_cast<std::string>(value) };
  }
}}  // namespace scenario_runner::utility

#endif  // SCENARIO_RUNNER__UTILITY__HIGHLIGHTER_HPP_
