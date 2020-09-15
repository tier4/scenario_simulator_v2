#ifndef SCENARIO_RUNNER__SYNTAX__CUSTOM_COMMAND_ACTION_HPP_
#define SCENARIO_RUNNER__SYNTAX__CUSTOM_COMMAND_ACTION_HPP_

#include <scenario_runner/reader/content.hpp>
#include <scenario_runner/syntax/command.hpp>

namespace scenario_runner { inline namespace syntax
{
  /* ==== CustomCommandAction ==================================================
   *
   * <xsd:complexType name="CustomCommandAction">
   *   <xsd:simpleContent>
   *     <xsd:extension base="xsd:string">
   *       <xsd:attribute name="type" type="String" use="required"/>
   *     </xsd:extension>
   *   </xsd:simpleContent>
   * </xsd:complexType>
   *
   * ======================================================================== */
  struct CustomCommandAction
  {
    const Command command;

    const String content;

    template <typename Node, typename Scope>
    explicit CustomCommandAction(const Node& node, Scope& scope)
      : command { readAttribute<Command>(node, scope, "type") }
      , content { readContent<String>(node, scope) }
    {}

    Object aux;

    auto accomplished()
    {
      switch (command)
      {
      case Command::debugAccomplishment:

        if (not aux)
        {
          aux = make(std::chrono::high_resolution_clock::now());
          return false;
        }
        else
        {
          const auto elapsed {
            std::chrono::duration_cast<std::chrono::seconds>(
              std::chrono::high_resolution_clock::now() - aux.as<decltype(std::chrono::high_resolution_clock::now())>(__FILE__, __LINE__))
          };

          return 3 < elapsed.count();
        }

      default:
        return false;
      }
    }

    auto evaluate()
    {
      switch (command)
      {
      case Command::debugAccomplishment:
        std::cout << *this << std::endl;
        return unspecified;

      case Command::print:
        std::cout << content << std::endl;
        return unspecified;

      default:
        throw command;
      }
    }

    template <typename... Ts>
    friend std::basic_ostream<Ts...>& operator <<(std::basic_ostream<Ts...>& os, const CustomCommandAction& action)
    {
      os << indent
         << blue << "<CustomCommandAction"
         << " " << highlight("type", action.command);

      switch (action.command)
      {
      case Command::print:
        return os << blue << ">" << reset
                  << action.content
                  << blue << "</CustomCommandAction>" << reset;

      case Command::debugAccomplishment:
      default:
        return os << blue << "/>" << reset;
      }
    }
  };
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__CUSTOM_COMMAND_ACTION_HPP_
