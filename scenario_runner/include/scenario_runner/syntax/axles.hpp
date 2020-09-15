#ifndef SCENARIO_RUNNER__SYNTAX__AXLES_HPP_
#define SCENARIO_RUNNER__SYNTAX__AXLES_HPP_

#include <scenario_runner/syntax/axle.hpp>

namespace scenario_runner { inline namespace syntax
{
  /* ==== Axles ================================================================
   *
   * <xsd:complexType name="Axles">
   *   <xsd:sequence>
   *     <xsd:element name="FrontAxle" type="Axle"/>
   *     <xsd:element name="RearAxle" type="Axle"/>
   *     <xsd:element name="AdditionalAxle" type="Axle" minOccurs="0" maxOccurs="unbounded"/>
   *   </xsd:sequence>
   * </xsd:complexType>
   *
   * ======================================================================== */
  struct Axles
  {
    const FrontAxle front_axle;

    const RearAxle rear_axle;

    std::vector<AdditionalAxle> additional_axles;

    Axles() = default;

    template <typename Node, typename Scope>
    explicit Axles(const Node& node, Scope& scope)
      : front_axle { readElement<FrontAxle>("FrontAxle", node, scope) }
      ,  rear_axle { readElement< RearAxle>( "RearAxle", node, scope) }
    {
      callWithElements(node, "AdditionalAxle", 0, unbounded, [&](auto&& node)
      {
        additional_axles.emplace_back(node, scope);
      });
    }
  };

  template <typename... Ts>
  std::basic_ostream<Ts...>& operator <<(std::basic_ostream<Ts...>& os, const Axles& rhs)
  {
    os << (indent++) << blue << "<Axles>\n" << reset
       << rhs.front_axle << "\n"
       << rhs.rear_axle << "\n";

    for (const auto& each : rhs.additional_axles)
    {
      os << each << "\n";
    }

    return os << (--indent) << blue << "</Axles>" << reset;
  }
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__AXLES_HPP_
