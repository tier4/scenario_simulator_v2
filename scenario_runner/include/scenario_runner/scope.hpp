#ifndef SCENARIO_RUNNER__SCOPE_HPP_
#define SCENARIO_RUNNER__SCOPE_HPP_

#include <scenario_runner/syntax/entity_ref.hpp>
// #include <simulation_controller/api/api.hpp>
#include <unordered_map>

namespace scenario_runner
{
  struct Scope
  {
    std::unordered_map<String, Object> parameters, entities, storyboard_elements;

    std::vector<EntityRef> actors;

    // std::shared_ptr<scenario_simulator::API> connection;

    explicit Scope() = delete;

    explicit Scope(      Scope&) = default;
    explicit Scope(const Scope&) = default;

    template <typename... Ts>
    explicit Scope(Ts&&...)
    {}

    // template <typename... Ts>
    // explicit Scope(Ts&&... xs)
    //   : connection { std::make_shared<scenario_simulator::API>(std::forward<decltype(xs)>(xs)...) }
    // {}

  public:
    // template <typename... Ts>
    // decltype(auto) getEntityStatus(Ts&&... xs) try
    // {
    //   return connection->entity->getEntityStatus(std::forward<decltype(xs)>(xs)...);
    // }
    // catch (const simulation_controller::SimulationRuntimeError& error)
    // {
    //   std::stringstream ss {};
    //   ss << error.what() << ".\n"
    //      << "Possible causes:\n"
    //      << "  (1) The position of the corresponding entity is not specified by Teleport Action";
    //   throw SemanticError { ss.str() };
    // }
    //
    // template <typename... Ts>
    // auto getDistanceAlongRoute(Ts&&... xs) const
    // {
    //   if (const auto result { connection->entity->getLongitudinalDistance(std::forward<decltype(xs)>(xs)...) })
    //   {
    //     return *result;
    //   }
    //   else
    //   {
    //     using value_type = typename std::decay<decltype(result)>::type::value_type;
    //     return std::numeric_limits<value_type>::infinity();
    //   }
    // }
    //
    // template <typename... Ts>
    // auto getTimeHeadway(Ts&&... xs) const
    // {
    //   if (const auto result { connection->entity->getTimeHeadway(std::forward<decltype(xs)>(xs)...) })
    //   {
    //     return *result;
    //   }
    //   else
    //   {
    //     using value_type = typename std::decay<decltype(result)>::type::value_type;
    //     return std::numeric_limits<value_type>::quiet_NaN();
    //   }
    // }
  };
}  // scenario_runner

#endif  // SCENARIO_RUNNER__SCOPE_HPP_
