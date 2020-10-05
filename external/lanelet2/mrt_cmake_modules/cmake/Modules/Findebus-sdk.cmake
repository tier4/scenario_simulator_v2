set(PACKAGE_HEADER_FILES PvBase.h)
# log4cxx

set(PACKAGE_LIBRARIES
    PvPersistence
    PtUtilsLib
    PtConvertersLib
    PvAppUtils
    PvCameraBridge
    EbTransportLayerLib
    PvGenICam
    PvBase
    PvSystem
    PvDevice
    PvVirtualDevice
    EbUtilsLib
    PvStream
    PvTransmitter
    XmlParser_gcc421_v3_0
    NodeMapData_gcc421_v3_0
    log4cpp_gcc421_v3_0
    Log_gcc421_v3_0
    MathParser_gcc421_v3_0
    GCBase_gcc421_v3_0
    GenApi_gcc421_v3_0
    PvSerial
    PvBuffer
    SimpleImagingLib)

set(PACKAGE_PATH /opt/pleora/ebus_sdk/Ubuntu-x86_64)
set(PACKAGE_PATH_GENI /opt/pleora/ebus_sdk/Ubuntu-x86_64/lib/genicam/bin/Linux64_x64)

set(ebus-sdk_INCLUDE_DIR "")
set(ebus-sdk_INCLUDE_DIR_VAR_NAMES "")
foreach(header_file ${PACKAGE_HEADER_FILES})
    find_path(
        ${header_file}_INCLUDE_DIR
        NAMES ${header_file}
        PATHS "${PACKAGE_PATH}/include")
    list(APPEND ebus-sdk_INCLUDE_DIR ${${header_file}_INCLUDE_DIR})
    list(APPEND ebus-sdk_INCLUDE_DIR_VAR_NAMES ${header_file}_INCLUDE_DIR)
endforeach()

set(ebus-sdk_LIBRARIES "")
set(ebus-sdk_LIBRARY_VAR_NAMES "")
foreach(library ${PACKAGE_LIBRARIES})
    find_library(
        ${library}_LIBRARY
        NAMES ${library}
        PATHS "${PACKAGE_PATH}/lib" "${PACKAGE_PATH_GENI}")
    list(APPEND ebus-sdk_LIBRARIES ${${library}_LIBRARY})
    list(APPEND ebus-sdk_LIBRARY_VAR_NAMES ${library}_LIBRARY)
endforeach()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    ebus-sdk
    FOUND_VAR ebus-sdk_FOUND
    REQUIRED_VARS ${ebus-sdk_INCLUDE_DIR_VAR_NAMES} ${ebus-sdk_LIBRARY_VAR_NAMES})
mark_as_advanced(${ebus-sdk_INCLUDE_DIR_VAR_NAMES} ${ebus-sdk_LIBRARY_VAR_NAMES})
