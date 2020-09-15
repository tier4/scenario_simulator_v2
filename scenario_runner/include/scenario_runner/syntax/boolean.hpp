#ifndef SCENARIO_RUNNER__SYNTAX__BOOLEAN_HPP_
#define SCENARIO_RUNNER__SYNTAX__BOOLEAN_HPP_

#include <boost/io/ios_state.hpp>
#include <iomanip>
#include <scenario_runner/object.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== Boolean ==============================================================
 *
 * TODO
 *
 * ======================================================================== */
struct Boolean
{
  using value_type = bool;

  value_type data;

  Boolean(value_type value = {}) noexcept
  {
    data = value;
  }

  Boolean(const std::string & target)
  {
    std::stringstream interpreter {};

    if (not (interpreter << target and interpreter >> std::boolalpha >> data)) {
      std::stringstream ss {};
      ss << "can't treat value " << std::quoted(target) << " as type Boolean";
      throw SyntaxError {ss.str()};
    }
  }

  operator value_type() const noexcept
  {
    return data;
  }
};

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(std::basic_ostream<Ts...> & os, const Boolean & boolean)
{
  boost::io::ios_flags_saver saver {os};
  return os << std::boolalpha << boolean.data;
}

template<typename ... Ts>
std::basic_istream<Ts...> & operator>>(std::basic_istream<Ts...> & is, Boolean & boolean)
{
  boost::io::ios_flags_saver saver {is};
  return is >> std::boolalpha >> boolean.data;
}

static const auto true_v  {make<Boolean>(true)};
static const auto false_v {make<Boolean>(false)};

auto asBoolean(bool value)
{
  return value ? true_v : false_v;
}
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__BOOLEAN_HPP_
