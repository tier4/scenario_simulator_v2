#ifndef SCENARIO_RUNNER__SYNTAX__DOUBLE_HPP_
#define SCENARIO_RUNNER__SYNTAX__DOUBLE_HPP_

#include <boost/lexical_cast.hpp>
#include <std_msgs/msg/float64.hpp>
#include <regex>

namespace scenario_runner { inline namespace syntax
{
  struct Double
    : public std_msgs::msg::Float64
  {
    using value_type = decltype(std_msgs::msg::Float64::data);

    Double(value_type value = {})
    {
      data = value;
    }

    explicit Double(const std::string& s) try
    {
      data = boost::lexical_cast<value_type>(s);
    }
    catch (const boost::bad_lexical_cast&)
    {
      std::stringstream ss {};
      ss << "can't treat value \"" << s << "\" as type Double";
      throw SyntaxError { ss.str() };
    }

    constexpr operator value_type() const noexcept
    {
      return data;
    }
  };

  template <typename... Ts>
  decltype(auto) operator <<(std::basic_ostream<Ts...>& os, const Double& rhs)
  {
    return os << std::fixed << rhs.data;
  }

  std::istream& operator >>(std::istream& is, Double& rhs)
  {
    std::string token {};

    is >> token;

    static const std::regex infinity { R"([+-]?inf(inity)?)" };

    std::smatch result {};

    if (std::regex_match(token, result, infinity))
    {
      #ifndef SCENARIO_RUNNER_ALLOW_INFINITY
      #define SCENARIO_RUNNER_DOUBLE_INFINITY max
      #else
      #define SCENARIO_RUNNER_DOUBLE_INFINITY infinity
      #endif

      rhs.data = (result.str(1) == "-" ? -1 : 1) * std::numeric_limits<Double::value_type>::SCENARIO_RUNNER_DOUBLE_INFINITY();

      #undef SCENARIO_RUNNER_DOUBLE_INFINITY
    }
    else
    {
      rhs.data = boost::lexical_cast<Double::value_type>(token);
    }

    return is;
  }
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__DOUBLE_HPP_
