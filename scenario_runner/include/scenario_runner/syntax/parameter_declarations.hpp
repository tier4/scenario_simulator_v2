#ifndef SCENARIO_RUNNER__SYNTAX__PARAMETER_DECLARATIONS_HPP_
#define SCENARIO_RUNNER__SYNTAX__PARAMETER_DECLARATIONS_HPP_

#include <scenario_runner/reader/element.hpp>
#include <scenario_runner/scope.hpp>
#include <scenario_runner/syntax/parameter_declaration.hpp>

namespace scenario_runner { inline namespace syntax
{
  /* ==== ParameterDeclarations ================================================
   *
   * <xsd:complexType name="ParameterDeclarations">
   *   <xsd:sequence>
   *     <xsd:element name="ParameterDeclaration" minOccurs="0" maxOccurs="unbounded" type="ParameterDeclaration"/>
   *   </xsd:sequence>
   * </xsd:complexType>
   *
   * ======================================================================== */
  struct ParameterDeclarations
    : public std::vector<ParameterDeclaration>
  {
    ParameterDeclarations() = default;

    template <typename Node, typename Scope>
    explicit ParameterDeclarations(const Node& node, Scope& scope)
    {
      callWithElements(node, "ParameterDeclaration", 0, unbounded, [&](auto&& each) mutable
      {
        return emplace_back(each, scope);
      });
    }
  };

  template <typename... Ts>
  std::basic_ostream<Ts...>& operator <<(std::basic_ostream<Ts...>& os, const ParameterDeclarations&)
  {
    return os << (indent++) << blue << "<ParameterDeclarations>" << reset << "\n"
              << (--indent) << blue << "</ParameterDeclarations>";
  }
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__PARAMETER_DECLARATIONS_HPP_
