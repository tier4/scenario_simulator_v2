#ifndef SCENARIO_RUNNER__SYNTAX__UNSIGNED_SHORT_HPP_
#define SCENARIO_RUNNER__SYNTAX__UNSIGNED_SHORT_HPP_

#include <boost/lexical_cast.hpp>
#include <std_msgs/msg/u_int16.hpp>

namespace scenario_runner
{inline namespace syntax
{
struct UnsignedShort
  : public std_msgs::msg::UInt16
{
  using value_type = decltype(std_msgs::msg::UInt16::data);

  UnsignedShort(value_type value = {})
  {
    data = value;
  }

  explicit UnsignedShort(const std::string & s) try
  {
    data = boost::lexical_cast<value_type>(s);
  } catch (const boost::bad_lexical_cast &) {
    std::stringstream ss {};
    ss << "can't treat value \"" << s << "\" as type UnsignedShort";
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
decltype(auto) operator<<(std::basic_ostream<Ts...>&os, const UnsignedShort & rhs)
{
  return os << rhs.data;
}

template<typename ... Ts>
decltype(auto) operator>>(std::basic_istream<Ts...>&is, UnsignedShort & rhs)
{
  return is >> rhs.data;
}
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__UNSIGNED_SHORT_HPP_
