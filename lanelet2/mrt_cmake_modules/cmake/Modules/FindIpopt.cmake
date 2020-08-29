# try to find ipopt
find_package(PkgConfig)
find_path(IPOPT_INCLUDE_DIRS IpIpoptApplication.hpp PATH_SUFFIXES coin)

find_library(IPOPT_LIBRARIES NAMES ipopt)
add_definitions(-DHAVE_STDDEF_H)
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Ipopt DEFAULT_MSG IPOPT_LIBRARIES IPOPT_INCLUDE_DIRS)

mark_as_advanced(IPOPT_LIBRARIES IPOPT_INCLUDE_DIRS)
