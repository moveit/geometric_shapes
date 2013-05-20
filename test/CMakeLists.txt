file(TO_NATIVE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/resources" TEST_RESOURCES_DIR)
if(WIN32)
    # Correct directory separator for Windows
    string(REPLACE "\\" "\\\\" TEST_RESOURCES_DIR "${TEST_RESOURCES_DIR}")
endif(WIN32)
configure_file("${TEST_RESOURCES_DIR}/config.h.in" "${TEST_RESOURCES_DIR}/config.h")
include_directories(.)
catkin_add_gtest(test_point_inclusion test_point_inclusion.cpp)
target_link_libraries(test_point_inclusion ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
