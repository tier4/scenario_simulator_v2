#include <openscenario_interpreter/reader/attribute.hpp>

namespace openscenario_interpreter
{
inline namespace reader
{
template auto readAttribute<Boolean, pugi::xml_node, Scope>(const std::string &, const pugi::xml_node &, const Scope &) -> Boolean;
template auto readAttribute<UnsignedInteger, pugi::xml_node, Scope>(const std::string &, const pugi::xml_node &, const Scope &) -> UnsignedInteger;
template auto readAttribute<Double, pugi::xml_node, Scope>(const std::string &, const pugi::xml_node &, const Scope &) -> Double;
template auto readAttribute<String, pugi::xml_node, Scope>(const std::string &, const pugi::xml_node &, const Scope &) -> String;
template auto readAttribute<syntax::Rule, pugi::xml_node, Scope>(const std::string &, const pugi::xml_node &, const Scope &) -> syntax::Rule;
}
}
