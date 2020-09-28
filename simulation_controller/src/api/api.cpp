#include <simulation_controller/api/api.hpp>

namespace scenario_simulator
{
    API::API(std::string address, int port)
    {
        entity_manager_ptr_ = std::shared_ptr<simulation_controller::entity::EntityManager>
            (new simulation_controller::entity::EntityManager(false));
        client_ptr_ = std::shared_ptr<XmlRpc::XmlRpcClient>
            (new XmlRpc::XmlRpcClient(address.c_str(), port));
        simulation = std::shared_ptr<SimulationAPIImpl>
            (new SimulationAPIImpl(client_ptr_, entity_manager_ptr_));
        entity = std::shared_ptr<EntityAPIImpl>
            (new EntityAPIImpl(client_ptr_, entity_manager_ptr_));
    }
}  // namespace scenario_simulator
