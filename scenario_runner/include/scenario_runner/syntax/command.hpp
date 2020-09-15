#ifndef SCENARIO_RUNNER__SYNTAX__COMMAND_HPP_
#define SCENARIO_RUNNER__SYNTAX__COMMAND_HPP_

#include <scenario_runner/object.hpp>

namespace scenario_runner { inline namespace syntax
{
  /* ==== Command ==============================================================
   *
   * TODO
   *
   * ======================================================================== */
  struct Command
  {
    enum value_type
    {
      debugAccomplishment,
      exitFailure,
      exitSuccess,
      print,
    }
    value;

    explicit Command() = default;

    explicit Command(value_type value)
      : value { value }
    {}

    operator value_type() const noexcept
    {
      return value;
    }

    decltype(auto) operator =(const value_type& rhs)
    {
      value = rhs;
      return *this;
    }
  };

  template <typename... Ts>
  std::basic_istream<Ts...>& operator >>(std::basic_istream<Ts...>& is, Command& command)
  {
    std::string buffer {};

    is >> buffer;

    #define BOILERPLATE(IDENTIFIER)                                            \
    if (buffer == #IDENTIFIER) do                                              \
    {                                                                          \
      command = Command::IDENTIFIER;                                           \
      return is;                                                               \
    } while (false)

    BOILERPLATE(debugAccomplishment);
    BOILERPLATE(exitFailure);
    BOILERPLATE(exitSuccess);
    BOILERPLATE(print);

    #undef BOILERPLATE

    std::stringstream ss {};
    ss << "unexpected value \'" << buffer << "\' specified as type Command";
    throw SyntaxError { ss.str() };
  }

  template <typename... Ts>
  std::basic_ostream<Ts...>& operator <<(std::basic_ostream<Ts...>& os, const Command& command)
  {
    switch (command)
    {
      #define BOILERPLATE(NAME) case Command::NAME: return os << #NAME;

      BOILERPLATE(debugAccomplishment);
      BOILERPLATE(exitFailure);
      BOILERPLATE(exitSuccess);
      BOILERPLATE(print);

      #undef BOILERPLATE

    default:
      std::stringstream ss {};
      ss << "enum class Command holds unexpected value " << static_cast<Command::value_type>(command);
      throw ImplementationFault { ss.str() };
    }
  }

}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__COMMAND_HPP_

