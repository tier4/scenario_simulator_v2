if(NOT MrtQt5_FIND_COMPONENTS)
    set(_components
        QtCore
        QtDBus
        QtGui
        QtNetwork
        QtTest
        QtWidgets
        QtXml)
else()
    set(_components ${MrtQt5_FIND_COMPONENTS})
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

foreach(_component ${_components})
    set(_libname Qt5${_component})
    if(MrtQt5_FIND_REQUIRED)
        find_package(${_libname} REQUIRED)
    else()
        find_package(${_libname} QUIET)
    endif()

    if(${_libname}_FOUND)
        message(STATUS "  Found ${_libname}")
        list(APPEND QT_INCLUDES ${${_libname}_INCLUDE_DIRS})
        list(APPEND QT_LIBRARIES "Qt5::${_component}")
    endif()
endforeach()
