#ifndef SCENARIO_RUNNER__SYNTAX__INTEGER_HPP_
#define SCENARIO_RUNNER__SYNTAX__INTEGER_HPP_

#include <boost/lexical_cast.hpp>
#include <std_msgs/msg/int32.hpp>

namespace scenario_runner { inline namespace syntax
{
  struct Integer
    : public std_msgs::msg::Int32
  {
    using value_type = decltype(std_msgs::msg::Int32::data);

    template <typename T>
    explicit constexpr Integer(T&& value)
    {
      data = value;
    }

    explicit Integer(const std::string& s) try
    {
      data = boost::lexical_cast<value_type>(s);
    }
    catch (const boost::bad_lexical_cast&)
    {
      std::stringstream ss {};
      ss << "can't treat value \"" << s << "\" as type Integer";
      throw SyntaxError { ss.str() };
    }

    constexpr operator value_type() const noexcept
    {
      return data;
    }
  };

  template <typename... Ts>
  decltype(auto) operator <<(std::basic_ostream<Ts...>& os, const Integer& rhs)
  {
    return os << rhs.data;
  }
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__INTEGER_HPP_
