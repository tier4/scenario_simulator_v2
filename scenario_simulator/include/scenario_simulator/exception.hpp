#ifndef SCENARIO_SIMULATOR__EXCEPTION_HPP_
#define SCENARIO_SIMULATOR__EXCEPTION_HPP_

namespace scenario_simulator
{
class SimulationRuntimeError : public std::runtime_error
{
public:
  SimulationRuntimeError(XmlRpc::XmlRpcValue value)
  : runtime_error(value["message"]) {}
  SimulationRuntimeError(const char * message)
  : runtime_error(message) {}

private:
};
}

#endif  // SCENARIO_SIMULATOR__EXCEPTION_HPP_
