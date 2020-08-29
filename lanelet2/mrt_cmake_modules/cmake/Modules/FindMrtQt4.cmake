find_package(Qt4 REQUIRED ${MrtQt4_FIND_COMPONENTS})
message(STATUS "Qt Components: " ${MrtQt4_FIND_COMPONENTS})
include(${QT_USE_FILE})
