#ifndef SCENARIO_RUNNER__SYNTAX__OBJECT_CONTROLLER_HPP_
#define SCENARIO_RUNNER__SYNTAX__OBJECT_CONTROLLER_HPP_

#include <scenario_runner/validator/attribute.hpp>
#include <scenario_runner/validator/choice.hpp>
#include <scenario_runner/validator/sequence.hpp>

namespace scenario_runner { inline namespace syntax
{
  /* ==== ObjectController =====================================================
   *
   * <xsd:complexType name="ObjectController">
   *   <xsd:choice>
   *     <xsd:element name="CatalogReference" type="CatalogReference"/>
   *     <xsd:element name="Controller" type="Controller"/>
   *   </xsd:choice>
   * </xsd:complexType>
   *
   * ======================================================================== */
  struct ObjectController
    : public Choice
  {
    template <typename... Ts>
    explicit ObjectController(Ts&&... xs)
    {
      defineElementAsUnsupported("CatalogReference", 0, 1);
      defineElementAsUnimplemented("Controller", 0, 1);

      validate(std::forward<decltype(xs)>(xs)...);
    }

    auto evaluate() const noexcept
    {
      return unspecified;
    }
  };
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__OBJECT_CONTROLLER_HPP_

