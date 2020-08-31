#ifndef LANELET2_EXTENSION__EXCEPTION_HPP
#define LANELET2_EXTENSION__EXCEPTION_HPP

#include <exception>
#include <string>

namespace lanelet
{
class HdMapException : public std::exception
{
public:
  explicit HdMapException(const std::string & msg)
  : error_message_(msg) {}
  virtual ~HdMapException() throw () {}
  virtual const char * what() const throw ()
  {
    return error_message_.c_str();
  }

protected:
  std::string error_message_;
};
}

#endif  // LANELET2_EXTENSION__EXCEPTION_HPP
