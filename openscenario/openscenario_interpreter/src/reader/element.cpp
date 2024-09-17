#include <openscenario_interpreter/reader/element.hpp>

namespace openscenario_interpreter
{
inline namespace reader
{
auto choice(const pugi::xml_node & node, std::unordered_map<std::string, std::function<Object(const pugi::xml_node &)>> callees) -> Object
{
  std::unordered_map<std::string, pugi::xml_node> specs{};

  for (const auto & each : callees) {
    if (const auto child = node.child(std::get<0>(each).c_str())) {
      specs.emplace(std::get<0>(each), child);
    }
  }

  auto print_keys_to = [&](auto & os, const auto & xs) -> decltype(auto) {
    if (not xs.empty()) {
      for (auto iter = std::begin(xs); iter != std::end(xs); ++iter) {
        os << std::get<0>(*iter);

        switch (std::distance(iter, std::end(xs))) {
          case 1:
            return os;

          case 2:
            os << " and ";
            break;

          default:
            os << ", ";
            break;
        }
      }
    }

    return os;
  };

  if (specs.empty()) {
    std::stringstream what;
    what << "Class " << node.name() << " requires one of following elements: ";
    print_keys_to(what, callees);
    what << ". But no element specified";
    throw SyntaxError(what.str());
  } else if (1 < specs.size()) {
    std::stringstream what;
    what << "Class " << node.name() << " requires just one of following elements: ";
    print_keys_to(what, callees);
    what << ". But " << specs.size() << " element" << (1 < specs.size() ? "s" : "") << " (";
    print_keys_to(what, specs);
    what << ") specified";
    throw SyntaxError(what.str());
  } else {
    const auto iter = std::cbegin(specs);
    return callees.at(std::get<0>(*iter))(std::get<1>(*iter));
  }
}
}  // namespace reader
}  // namespace openscenario_interpreter
