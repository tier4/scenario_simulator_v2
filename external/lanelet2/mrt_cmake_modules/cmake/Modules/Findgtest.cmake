if(CATKIN_ENABLE_TESTING)
    #just add gtest and gtest_main to link to
    #those files are generated during catkin test ny building gtest
    set(gtest_INCLUDE_DIRS "")
    set(gtest_LIBRARIES gtest)
else()
    message(FATAL_ERROR "CMake script only implemented for gtest as test_depend")
endif()
