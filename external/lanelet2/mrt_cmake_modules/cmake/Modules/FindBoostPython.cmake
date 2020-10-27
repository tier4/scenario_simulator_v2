# - Try to find the a valid boost+python combination
# Once done this will define
#
#  BoostPython_FOUND - system has a valid boost+python combination
#  BoostPython_INCLUDE_DIRS - the include directory for boost+python
#  BoostPython_LIBRARIES - the needed libs for boost+python

if(PYTHON_VERSION)
    set(_python_version ${PYTHON_VERSION})
else()
    set(_python_version 2.7)
endif()
if(_python_version VERSION_EQUAL 3 AND CMAKE_VERSION VERSION_GREATER 3.15)
    # we also need the subversion
    find_package(Python3 REQUIRED)
    set(_python_version ${Python3_VERSION})
endif()

# this only works with a recent cmake/boost combination
if(CMAKE_VERSION VERSION_GREATER 3.11)
    find_package(Boost COMPONENTS python${_python_version} numpy${_python_version})
    # ...unless we are dealing with boost 1.70 and above where we can only hope this finds the right version
    if(NOT Boost_PYTHON${_python_version}_FOUND)
        set(Boost_PYTHON_VERSION ${_python_version})
        find_package(Boost COMPONENTS python numpy)
        set(Boost_PYTHON${_python_version}_FOUND ${Boost_PYTHON_FOUND})
        set(Boost_PYTHON${_python_version}_LIBRARY ${Boost_PYTHON_LIBRARY})
        set(Boost_NUMPY${_python_version}_LIBRARY ${Boost_NUMPY_LIBRARY})
    endif()
endif()

if(NOT (Boost_PYTHON${_python_version}_FOUND OR Boost_python_FOUND))
    # on older cmake versions, "python" finds python2, otherwise python3
    set(_search_version)
    if(NOT _python_version VERSION_LESS 3)
        set(_search_version ${_python_version})
    endif()
    find_package(Boost REQUIRED COMPONENTS python${_search_version})
    find_package(Boost QUIET COMPONENTS numpy${_search_version}) # numpy is not available on some boost versions
    set(Python_ADDITIONAL_VERSIONS ${_python_version})
    find_package(PythonLibs REQUIRED)
    set(BoostPython_INCLUDE_DIRS ${Boost_INCLUDE_DIR} ${PYTHON_INCLUDE_DIR})
    set(BoostPython_LIBRARIES ${Boost_PYTHON${_search_version}_LIBRARIES} ${Boost_NUMPY${_search_version}_LIBRARIES}
                              ${PYTHON_LIBRARIES})
elseif(_python_version VERSION_LESS 3)
    find_package(Python2 REQUIRED COMPONENTS Development)
    set(BoostPython_INCLUDE_DIRS ${Boost_INCLUDE_DIR} ${Python2_INCLUDE_DIRS})
    set(BoostPython_LIBRARIES ${Boost_PYTHON${_python_version}_LIBRARY} ${Boost_NUMPY${_python_version}_LIBRARY}
                              ${Python2_LIBRARIES})
else()
    find_package(Python3 REQUIRED COMPONENTS Development)
    set(BoostPython_INCLUDE_DIRS ${Boost_INCLUDE_DIR} ${Python3_INCLUDE_DIRS})
    set(BoostPython_LIBRARIES ${Boost_PYTHON${_python_version}_LIBRARY} ${Boost_NUMPY${_python_version}_LIBRARY}
                              ${Python3_LIBRARIES})
endif()

find_package_handle_standard_args(BoostPython DEFAULT_MSG BoostPython_LIBRARIES BoostPython_INCLUDE_DIRS)
