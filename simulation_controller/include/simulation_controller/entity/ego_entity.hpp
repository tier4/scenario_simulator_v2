#ifndef SIMULATION_CONTROLLER__EGO_ENTITY_HPP_
#define SIMULATION_CONTROLLER__EGO_ENTITY_HPP_

#include <simulation_controller/entity/vehicle_entity.hpp>

// headers in pugixml
#include "pugixml.hpp"

namespace simulation_controller
{
    namespace entity
    {
        class EgoEntity : public VehicleEntity
        {
        public:
            EgoEntity(std::string name, const EntityStatus &initial_state, const pugi::xml_node & xml);
            EgoEntity(std::string name, const EntityStatus &initial_state, VehicleParameters parameters);
            EgoEntity(std::string name, const pugi::xml_node & xml);
            EgoEntity(std::string name, VehicleParameters parameters);
        };
    }  // namespace entity
}  // namespace simulation_controller

#endif  // SIMULATION_CONTROLLER__EGO_ENTITY_HPP_
