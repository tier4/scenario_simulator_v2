#ifndef SCENARIO_RUNNER__ERROR_HPP_
#define SCENARIO_RUNNER__ERROR_HPP_

#include <sstream>
#include <stdexcept>
#include <string>

namespace scenario_runner
{
  /* ==== Error ================================================================
   *
   * -- Error
   *     |-- SyntaxError
   *     |-- ConnectionError
   *     |-- SemanticError
   *     `-- ImplementationFault
   *
   * ======================================================================== */
  struct Error
    : public std::runtime_error
  {
    using std::runtime_error::runtime_error;
  };

  struct SyntaxError
    : public Error
  {
    explicit SyntaxError(const std::string& s)
      : Error { "syntax-error: " + s }
    {}
  };

  struct SemanticError
    : public Error
  {
    explicit SemanticError(const std::string& s)
      : Error { "semantic-error: " + s }
    {}
  };

  struct ConnectionError
    : public Error
  {
    explicit ConnectionError(const std::string& s)
      : Error { "connection-error: " + s }
    {}
  };

  struct ImplementationFault
    : public Error
  {
    explicit ImplementationFault(const std::string& s)
      : Error { "implementation-fault: " + s }
    {}
  };

  #define THROW(TYPENAME)                                                      \
  do                                                                           \
  {                                                                            \
    std::stringstream ss {};                                                   \
    ss << __FILE__ << ":" << __LINE__;                                         \
    throw TYPENAME { ss.str() };                                               \
  } while (false)

  #define THROW_IMPLEMENTATION_FAULT() THROW(ImplementationFault)

  #define UNIMPLEMENTED(NAME)                                                  \
  do                                                                           \
  {                                                                            \
    std::stringstream ss {};                                                   \
    ss << "given class \'" << NAME << "\' is valid OpenSCENARIO element, but is not yet implemented"; \
    throw ImplementationFault { ss.str() };                                    \
  } while (false)
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__ERROR_HPP_
