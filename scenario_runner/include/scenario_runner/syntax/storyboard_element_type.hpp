#ifndef SCEANRIO_RUNNER__SYNTAX__STORYBOARD_ELEMENT_TYPE_HPP_
#define SCEANRIO_RUNNER__SYNTAX__STORYBOARD_ELEMENT_TYPE_HPP_

namespace scenario_runner
{inline namespace syntax
{
/* ==== StoryboardElementType ================================================
 *
 * <xsd:simpleType name="StoryboardElementType">
 *   <xsd:union>
 *     <xsd:simpleType>
 *       <xsd:restriction base="xsd:string">
 *         <xsd:enumeration value="story"/>
 *         <xsd:enumeration value="act"/>
 *         <xsd:enumeration value="maneuver"/>
 *         <xsd:enumeration value="event"/>
 *         <xsd:enumeration value="action"/>
 *         <xsd:enumeration value="maneuverGroup"/>
 *       </xsd:restriction>
 *     </xsd:simpleType>
 *     <xsd:simpleType>
 *       <xsd:restriction base="parameter"/>
 *     </xsd:simpleType>
 *   </xsd:union>
 * </xsd:simpleType>
 *
 * ======================================================================== */
struct StoryboardElementType
{
  enum value_type
  {
    act,
    action,
    event,
    maneuver,
    maneuverGroup,
    story,
  } value;

  explicit constexpr StoryboardElementType(value_type = {})
  : value{value}
  {}

  constexpr operator value_type() const noexcept
  {
    return value;
  }
};

template<typename ... Ts>
std::basic_istream<Ts...> & operator>>(std::basic_istream<Ts...> & is, StoryboardElementType & type)
{
  std::string buffer {};

  is >> buffer;

    #define BOILERPLATE(IDENTIFIER) \
  if (buffer == #IDENTIFIER) do \
    { \
      type.value = StoryboardElementType::IDENTIFIER; \
      return is; \
    } while (false)

  BOILERPLATE(act);
  BOILERPLATE(action);
  BOILERPLATE(event);
  BOILERPLATE(maneuver);
  BOILERPLATE(maneuverGroup);
  BOILERPLATE(story);

    #undef BOILERPLATE

  std::stringstream ss {};
  ss << "unexpected value \'" << buffer << "\' specified as type StoryboardElementType";
  throw SyntaxError {ss.str()};
}

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(
  std::basic_ostream<Ts...> & os,
  const StoryboardElementType & type)
{
  switch (type) {
      #define BOILERPLATE(IDENTIFIER) case StoryboardElementType::IDENTIFIER: return os << \
           #IDENTIFIER;

    BOILERPLATE(act);
    BOILERPLATE(action);
    BOILERPLATE(event);
    BOILERPLATE(maneuver);
    BOILERPLATE(maneuverGroup);
    BOILERPLATE(story);

      #undef BOILERPLATE

    default:
      std::stringstream ss {};
      ss << "enum class StoryboardElementType holds unexpected value " <<
        static_cast<StoryboardElementType::value_type>(type.value);
      throw ImplementationFault {ss.str()};
  }
}
}}  // namespace scenario_runner::syntax

#endif  // SCEANRIO_RUNNER__SYNTAX__STORYBOARD_ELEMENT_TYPE_HPP_
