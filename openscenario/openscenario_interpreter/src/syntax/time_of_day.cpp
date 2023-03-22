#include "openscenario_interpreter/reader/attribute.hpp"

#include <openscenario_interpreter/syntax/time_of_day.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{

TimeOfDay::TimeOfDay(const pugi::xml_node & node, Scope & scope)
: animation(readAttribute<Boolean>("animation", node, scope)),
  dateTime(readAttribute<String>("dateTime", node, scope))
{
}

}  // namespace syntax
}  // namespace openscenario_interpreter
