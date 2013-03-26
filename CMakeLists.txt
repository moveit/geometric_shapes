cmake_minimum_required(VERSION 2.8.3)
project(geometric_shapes)

if (NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system filesystem)

find_package(ASSIMP QUIET)
if (NOT ASSIMP_FOUND)
  pkg_check_modules(ASSIMP assimp)
endif()
if (ASSIMP_FOUND)
  if( ${ASSIMP_VERSION} STRGREATER "2.0.0" )
    set(IS_ASSIMP3 1)
    add_definitions(-DIS_ASSIMP3)
    message(STATUS "Found assimp v3")
  else()
    message(STATUS "Found assimp v2")
  endif()
else()
  message(STATUS "could not find assimp (perhaps available thorugh ROS package?), so assuming assimp v2")
  set(ASSIMP_INCLUDE_DIRS )
  ste(ASSIMP_LIBRARY_DIRS )
  set(ASSIMP_LIBRARIES assimp)
  set(ASSIMP_LIBRARY_DIRS)
  set(ASSIMP_CXX_FLAGS)
  set(ASSIMP_CFLAGS_OTHER)
  set(ASSIMP_LINK_FLAGS)
  set(ASSIMP_INCLUDE_DIRS)
  set(IS_ASSIMP3 0) # most likely not
endif()

find_library(QHULL_LIB qhull)
find_package(octomap REQUIRED)
find_package(catkin COMPONENTS shape_msgs resource_retriever shape_tools random_numbers console_bridge eigen_stl_containers)

catkin_package(
  INCLUDE_DIRS include ${OCTOMAP_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME} ${OCTOMAP_LIBRARIES}
  CATKIN_DEPENDS console_bridge shape_tools shape_msgs random_numbers eigen_stl_containers
  DEPENDS Eigen
  )

include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${ASSIMP_INCLUDE_DIRS})
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} ${ASSIMP_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
  src/shapes.cpp
  src/shape_operations.cpp
  src/mesh_operations.cpp
  src/bodies.cpp
  src/body_operations.cpp)
target_link_libraries(${PROJECT_NAME} ${ASSIMP_LIBRARIES} ${QHULL_LIB} ${catkin_LIBRARIES} ${Boost_LIBRARIES})


# Unit tests
catkin_add_gtest(test_point_inclusion test/test_point_inclusion.cpp)
target_link_libraries(test_point_inclusion ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})


install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h")

