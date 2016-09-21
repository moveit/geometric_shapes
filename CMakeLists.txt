cmake_minimum_required(VERSION 2.8.12)
project(geometric_shapes)

add_compile_options(-std=c++11)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

if (NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: Choosing 'Release' for maximum performance")
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ASSIMP QUIET)
if (NOT ASSIMP_FOUND)
  find_package(PkgConfig REQUIRED)
  # assimp is required, so REQUIRE the second attempt
  pkg_check_modules(ASSIMP REQUIRED assimp)
endif()

find_package(Boost REQUIRED system filesystem)

find_package(console_bridge REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(octomap REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  eigen_stl_containers
  random_numbers
  resource_retriever
  shape_msgs
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES ${PROJECT_NAME} ${OCTOMAP_LIBRARIES}
  CATKIN_DEPENDS
    eigen_stl_containers
    random_numbers
    shape_msgs
    visualization_msgs
  DEPENDS
    EIGEN3
    OCTOMAP
    console_bridge
  )

find_package(Qhull REQUIRED)
if (HAVE_QHULL_2011)
  add_definitions(-DGEOMETRIC_SHAPES_HAVE_QHULL_2011)
endif()

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIR} ${ASSIMP_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS} ${console_bridge_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/bodies.cpp
  src/body_operations.cpp
  src/mesh_operations.cpp
  src/shape_extents.cpp
  src/shape_operations.cpp
  src/shape_to_marker.cpp
  src/shapes.cpp
)

target_link_libraries(${PROJECT_NAME} ${ASSIMP_LIBRARIES} ${QHULL_LIBRARIES} ${catkin_LIBRARIES} ${console_bridge_LIBRARIES} ${Boost_LIBRARIES})


if(CATKIN_ENABLE_TESTING)
  # Unit tests
  add_subdirectory(test)
endif()

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
