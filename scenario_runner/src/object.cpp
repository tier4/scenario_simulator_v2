#include <scenario_runner/object.hpp>

namespace scenario_runner
{
const Object unspecified {make<Unspecified>()};

std::ostream & operator<<(std::ostream & os, const Unspecified &)
{
  return os << "<!-- Unspecified -->";
}
}  // namespace scenario_runner
