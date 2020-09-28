#ifndef SIMULATION_CONTROLLER__EXCEPTION_HPP_
#define SIMULATION_CONTROLLER__EXCEPTION_HPP_

namespace simulation_controller
{
    class SimulationRuntimeError : public std::runtime_error
    {
    public:
        SimulationRuntimeError(const char *message) : runtime_error(message) {};
        SimulationRuntimeError(std::string message) : runtime_error(message.c_str()) {};
    };

    class SplineInterpolationError : public std::runtime_error
    {
    public:
        SplineInterpolationError(const char *message) : runtime_error(message) {};
    };
}  // namespace simulation_controller


#endif  // SIMULATION_CONTROLLER__EXCEPTION_HPP_
