#ifndef SIMULATION_CONTROLLER__EXCEPTION_HPP_
#define SIMULATION_CONTROLLER__EXCEPTION_HPP_

namespace simulation_controller
{
    class SimulationRuntimeError : public std::runtime_error
    {
    public:
        SimulationRuntimeError(const char *message, int res=0) : error_info_(res), runtime_error(message) {};
        SimulationRuntimeError(std::string message, int res=0) : error_info_(res), runtime_error(message.c_str()) {};
    private:
        int error_info_;
    };

    class SplineInterpolationError : public std::runtime_error
    {
    public:
        SplineInterpolationError(const char *message, int res=0) : error_info_(res), runtime_error(message) {};
    private:
        int error_info_;
    };
}  // namespace simulation_controller


#endif  // SIMULATION_CONTROLLER__EXCEPTION_HPP_
