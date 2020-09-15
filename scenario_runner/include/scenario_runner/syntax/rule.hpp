#ifndef SCENARIO_RUNNER__SYNTAX__RULE_HPP_
#define SCENARIO_RUNNER__SYNTAX__RULE_HPP_

#include <functional>
#include <scenario_runner/reader/attribute.hpp>

namespace scenario_runner
{inline namespace syntax
{
template<typename T, typename = void>
struct equal_to
  : public std::equal_to<T>
{};

template<typename T>
struct equal_to<T, typename std::enable_if<not std::numeric_limits<T>::is_integer>::type>
{
  constexpr auto operator()(const T & lhs, const T & rhs) const noexcept
  {
    return std::abs(lhs - rhs) < std::numeric_limits<typename std::decay<T>::type>::epsilon();
  }
};

/* ==== Rule =================================================================
 *
 * <xsd:simpleType name="Rule">
 *   <xsd:union>
 *     <xsd:simpleType>
 *       <xsd:restriction base="xsd:string">
 *         <xsd:enumeration value="greaterThan"/>
 *         <xsd:enumeration value="lessThan"/>
 *         <xsd:enumeration value="equalTo"/>
 *       </xsd:restriction>
 *     </xsd:simpleType>
 *     <xsd:simpleType>
 *       <xsd:restriction base="parameter"/>
 *     </xsd:simpleType>
 *   </xsd:union>
 * </xsd:simpleType>
 *
 * ======================================================================== */
struct Rule
{
  enum value_type
  {
    greaterThan, lessThan, equalTo,
  } value;

  explicit constexpr Rule(value_type value = {})
  : value{value}
  {}

  constexpr operator value_type() const noexcept
  {
    return value;
  }

  template<typename T, typename U = T>
  constexpr decltype(auto) operator()(T && lhs, U && rhs) const noexcept
  {
    switch (value) {
      case greaterThan:
        return std::greater<void>()(std::forward<decltype(lhs)>(lhs),
                 std::forward<decltype(rhs)>(rhs));

      case lessThan:
        return std::less<void>()(std::forward<decltype(lhs)>(lhs),
                 std::forward<decltype(rhs)>(rhs));

      case equalTo:
        return equal_to<T>()(std::forward<decltype(lhs)>(lhs), std::forward<decltype(rhs)>(rhs));

      default:
        return false;
    }
  }
};

template<typename ... Ts>
std::basic_istream<Ts...> & operator>>(std::basic_istream<Ts...> & is, Rule & rule)
{
  std::string buffer {};

  is >> buffer;

    #define BOILERPLATE(IDENTIFIER) \
  if (buffer == #IDENTIFIER) do \
    { \
      rule.value = Rule::IDENTIFIER; \
      return is; \
    } while (false)

  BOILERPLATE(greaterThan);
  BOILERPLATE(lessThan);
  BOILERPLATE(equalTo);

    #undef BOILERPLATE

  std::stringstream ss {};
  ss << "unexpected value \'" << buffer << "\' specified as type Rule";
  throw SyntaxError {ss.str()};
}

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(std::basic_ostream<Ts...> & os, const Rule & rule)
{
  switch (rule) {
      #define BOILERPLATE(IDENTIFIER) case Rule::IDENTIFIER: return os << #IDENTIFIER;

    BOILERPLATE(greaterThan);
    BOILERPLATE(lessThan);
    BOILERPLATE(equalTo);

      #undef BOILERPLATE

    default:
      std::stringstream ss {};
      ss << "enum class Rule holds unexpected value " << static_cast<Rule::value_type>(rule.value);
      throw ImplementationFault {ss.str()};
  }
}
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__RULE_HPP_
