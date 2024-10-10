#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/rule.hpp>

namespace openscenario_interpreter
{
template auto reader::substitute(std::string, const Scope &) -> String;

template auto reader::readAttribute(const std::string &, const pugi::xml_node &, const Scope &)
  -> Boolean;
template auto reader::readAttribute(const std::string &, const pugi::xml_node &, const Scope &)
  -> UnsignedShort;
template auto reader::readAttribute(const std::string &, const pugi::xml_node &, const Scope &)
  -> UnsignedInteger;
template auto reader::readAttribute(const std::string &, const pugi::xml_node &, const Scope &)
  -> Double;
template auto reader::readAttribute(const std::string &, const pugi::xml_node &, const Scope &)
  -> String;
template auto reader::readAttribute(const std::string &, const pugi::xml_node &, const Scope &)
  -> syntax::Rule;
}  // namespace openscenario_interpreter
