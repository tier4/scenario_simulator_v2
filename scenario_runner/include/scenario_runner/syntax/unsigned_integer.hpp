#ifndef SCENARIO_RUNNER__SYNTAX__UNSIGNED_INTEGER_HPP_
#define SCENARIO_RUNNER__SYNTAX__UNSIGNED_INTEGER_HPP_

#include <boost/lexical_cast.hpp>
#include <std_msgs/msg/u_int64.hpp>

namespace scenario_runner
{inline namespace syntax
{
struct UnsignedInteger
  : public std_msgs::msg::UInt64
{
  using value_type = decltype(std_msgs::msg::UInt64::data);

  UnsignedInteger(value_type value = {})
  {
    data = value;
  }

  explicit UnsignedInteger(const std::string & s) try
  {
    data = boost::lexical_cast<value_type>(s);
  } catch (const boost::bad_lexical_cast &) {
    std::stringstream ss {};
    ss << "can't treat value \"" << s << "\" as type UnsignedInteger";
    throw SyntaxError {ss.str()};
  }

  constexpr operator value_type() const noexcept
  {
    return data;
  }

  decltype(auto) operator++() noexcept
  {
    ++data;
    return *this;
  }
};

template<typename ... Ts>
decltype(auto) operator<<(std::basic_ostream<Ts...>&os, const UnsignedInteger & rhs)
{
  return os << rhs.data;
}

template<typename ... Ts>
decltype(auto) operator>>(std::basic_istream<Ts...>&is, UnsignedInteger & rhs)
{
  return is >> rhs.data;
}

using UnsignedInt = UnsignedInteger;
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__UNSIGNED_INTEGER_HPP_
