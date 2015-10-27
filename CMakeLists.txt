cmake_minimum_required(VERSION 2.8.3)
project(geometric_shapes)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/" ${CMAKE_MODULE_PATH})

if (NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED system filesystem)

find_package(ASSIMP QUIET)
if (NOT ASSIMP_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(ASSIMP assimp)
endif()
if (ASSIMP_FOUND)
  if (NOT ${ASSIMP_VERSION} VERSION_LESS "2.0.1150")
    add_definitions(-DASSIMP_UNIFIED_HEADER_NAMES)
    message(STATUS "Assimp version has unified headers")
  else()
    message(STATUS "Assimp version does not have unified headers")
  endif()
else()
  message(STATUS "could not find assimp (perhaps available thorugh ROS package?), so assuming assimp v2")
  set(ASSIMP_INCLUDE_DIRS )
  set(ASSIMP_LIBRARY_DIRS )
  set(ASSIMP_LIBRARIES assimp)
  set(ASSIMP_LIBRARY_DIRS)
  set(ASSIMP_CXX_FLAGS)
  set(ASSIMP_CFLAGS_OTHER)
  set(ASSIMP_LINK_FLAGS)
  set(ASSIMP_INCLUDE_DIRS)
endif()

find_package(octomap REQUIRED)
find_package(catkin COMPONENTS cmake_modules visualization_msgs shape_msgs resource_retriever random_numbers eigen_stl_containers)
find_package(console_bridge REQUIRED)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${OCTOMAP_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME} ${OCTOMAP_LIBRARIES}
  CATKIN_DEPENDS shape_msgs random_numbers eigen_stl_containers visualization_msgs
  DEPENDS Eigen console_bridge
  )

find_package(Qhull REQUIRED)
if (HAVE_QHULL_2011)
  add_definitions(-DGEOMETRIC_SHAPES_HAVE_QHULL_2011)
endif()

include_directories(include)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${ASSIMP_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS} ${console_bridge_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} ${console_bridge_LIBRARY_DIRS} ${ASSIMP_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
  src/shapes.cpp
  src/shape_operations.cpp
  src/mesh_operations.cpp
  src/bodies.cpp
  src/body_operations.cpp
  src/shape_to_marker.cpp
  src/shape_extents.cpp)
target_link_libraries(${PROJECT_NAME} ${ASSIMP_LIBRARIES} ${QHULL_LIBRARIES} ${catkin_LIBRARIES} ${console_bridge_LIBRARIES} ${Boost_LIBRARIES})


if(CATKIN_ENABLE_TESTING)
  # Unit tests
  add_subdirectory(test)
endif()

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h")