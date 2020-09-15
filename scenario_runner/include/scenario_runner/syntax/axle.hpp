#ifndef SCENARIO_RUNNER__SYNTAX__AXLE_HPP_
#define SCENARIO_RUNNER__SYNTAX__AXLE_HPP_

#include <scenario_runner/reader/attribute.hpp>
#include <scenario_runner/reader/element.hpp>

namespace scenario_runner { inline namespace syntax
{
  /* ==== Axle =================================================================
   *
   * <xsd:complexType name="Axle">
   *   <xsd:attribute name="maxSteering" type="Double" use="required"/>
   *   <xsd:attribute name="wheelDiameter" type="Double" use="required"/>
   *   <xsd:attribute name="trackWidth" type="Double" use="required"/>
   *   <xsd:attribute name="positionX" type="Double" use="required"/>
   *   <xsd:attribute name="positionZ" type="Double" use="required"/>
   * </xsd:complexType>
   *
   * ======================================================================== */
  struct Axle
  {
    const Double max_steering, wheel_diameter, track_width, position_x, position_z;

    Axle() = default;

    template <typename Node, typename Scope>
    explicit Axle(const Node& node, Scope& scope)
      : max_steering   { readAttribute<Double>(node, scope, "maxSteering") }
      , wheel_diameter { readAttribute<Double>(node, scope, "wheelDiameter") }
      , track_width    { readAttribute<Double>(node, scope, "trackWidth") }
      , position_x     { readAttribute<Double>(node, scope, "positionX") }
      , position_z     { readAttribute<Double>(node, scope, "positionZ") }
    {}
  };

  #define BOILERPLATE(TYPENAME)                                                \
  template <typename... Ts>                                                    \
  std::basic_ostream<Ts...>& operator <<(std::basic_ostream<Ts...>& os, const TYPENAME& rhs) \
  {                                                                            \
    return os << indent << blue << "<" #TYPENAME << " " << highlight("maxSteering",   rhs.max_steering) \
                                                 << " " << highlight("wheelDiameter", rhs.wheel_diameter) \
                                                 << " " << highlight("trackWidth",    rhs.track_width) \
                                                 << " " << highlight("positionX",     rhs.position_x) \
                                                 << " " << highlight("positionZ",     rhs.position_z) << blue << "/>" << reset; \
  } static_assert(true, "")

  BOILERPLATE(Axle);

  // NOTE: DON'T REWRITE THIS STRUCT LIKE `using FrontAxle = Axle` (for Clang)
  struct FrontAxle
    : public Axle
  {
    using Axle::Axle;
  };

  BOILERPLATE(FrontAxle);

  // NOTE: DON'T REWRITE THIS STRUCT LIKE `using RearAxle = Axle` (for Clang)
  struct RearAxle
    : public Axle
  {
    using Axle::Axle;
  };

  BOILERPLATE(RearAxle);

  // NOTE: DON'T REWRITE THIS STRUCT LIKE `using AdditionalAxle = Axle` (for Clang)
  struct AdditionalAxle
    : public Axle
  {
    using Axle::Axle;
  };

  BOILERPLATE(AdditionalAxle);

  #undef BOILERPLATE
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__AXLE_HPP_
