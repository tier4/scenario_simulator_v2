# These tests have to be separate, because one requires including operator.hpp and other requires not including operator.hpp
ament_add_gtest(test_truncate
    test_vector3.cpp
    test_truncate_msg.cpp
    test_truncate_custom.cpp)
target_link_libraries(test_truncate geometry)
