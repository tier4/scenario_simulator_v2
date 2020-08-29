# libigl must be set up manually in src-folder of workspace via:
#  git clone https://github.com/libigl/libigl.git
#  cd libigl
#  git submodule update --init --recursive
#  mkdir build
#  cd build
#  cmake ..
set(IGL_FOUND FALSE)

find_path(
    LIBIGL_INCLUDE_DIR igl/readOBJ.h
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../libigl
    PATH_SUFFIXES include)

if(${LIBIGL_INCLUDE_DIR} STREQUAL "LIBIGL_INCLUDE_DIR-NOTFOUND")
    return()
endif()

option(LIBIGL_USE_STATIC_LIBRARY "Use libigl as static library" OFF)
option(LIBIGL_WITH_CGAL "Use CGAL" OFF)
option(LIBIGL_WITH_COMISO "Use CoMiso" OFF)
option(LIBIGL_WITH_CORK "Use Cork" OFF)
option(LIBIGL_WITH_EMBREE "Use Embree" OFF)
option(LIBIGL_WITH_MATLAB "Use Matlab" OFF)
option(LIBIGL_WITH_MOSEK "Use MOSEK" OFF)
option(LIBIGL_WITH_OPENGL "Use OpenGL" ON)
option(LIBIGL_WITH_OPENGL_GLFW "Use GLFW" ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use ImGui" OFF)
option(LIBIGL_WITH_PNG "Use PNG" ON)
option(LIBIGL_WITH_TETGEN "Use Tetgen" OFF)
option(LIBIGL_WITH_TRIANGLE "Use Triangle" OFF)
option(LIBIGL_WITH_XML "Use XML" OFF)
option(LIBIGL_WITH_PYTHON "Use Python" OFF)
option(LIBIGL_WITHOUT_COPYLEFT "Disable Copyleft libraries" OFF)

list(APPEND CMAKE_MODULE_PATH "${LIBIGL_INCLUDE_DIR}/../cmake")
include(libigl)

set(IGL_LIBRARIES igl::core igl::opengl_glfw igl::png)
set(IGL_FOUND TRUE)
