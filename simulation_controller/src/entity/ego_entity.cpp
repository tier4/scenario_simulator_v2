#include <simulation_controller/entity/ego_entity.hpp>

namespace simulation_controller
{
namespace entity
{
EgoEntity::EgoEntity(
  std::string name, const EntityStatus & initial_state,
  const pugi::xml_node & xml)
: VehicleEntity(name, initial_state, xml) {}
EgoEntity::EgoEntity(
  std::string name, const EntityStatus & initial_state,
  VehicleParameters parameters)
: VehicleEntity(name, initial_state, parameters) {}
EgoEntity::EgoEntity(std::string name, const pugi::xml_node & xml)
: VehicleEntity(name, xml) {}
EgoEntity::EgoEntity(std::string name, VehicleParameters parameters)
: VehicleEntity(name, parameters) {}
}      // namespace entity
} // namespace simulation_controller
