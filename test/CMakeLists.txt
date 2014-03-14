file(TO_NATIVE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/resources" TEST_RESOURCES_DIR)
if(WIN32)
    # Correct directory separator for Windows
    string(REPLACE "\\" "\\\\" TEST_RESOURCES_DIR "${TEST_RESOURCES_DIR}")
endif(WIN32)
configure_file(resources/config.h.in "${CMAKE_CURRENT_BINARY_DIR}/resources/config.h")
include_directories(${CMAKE_CURRENT_BINARY_DIR})

catkin_add_gtest(test_point_inclusion test_point_inclusion.cpp)
target_link_libraries(test_point_inclusion ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_add_gtest(test_bounding_sphere test_bounding_sphere.cpp)
target_link_libraries(test_bounding_sphere ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
