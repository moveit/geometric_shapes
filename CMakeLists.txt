cmake_minimum_required(VERSION 3.5)
project(geometric_shapes)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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

find_package(shape_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED system filesystem)
find_package(console_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(eigen_stl_containers REQUIRED)
find_package(random_numbers REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qhull REQUIRED)
if (HAVE_QHULL_2011)
  add_definitions(-DGEOMETRIC_SHAPES_HAVE_QHULL_2011)
endif()

# catkin_package(
#   INCLUDE_DIRS
#     include
#   LIBRARIES ${PROJECT_NAME} ${OCTOMAP_LIBRARIES}
#   CATKIN_DEPENDS
#     eigen_stl_containers
#     random_numbers
#     shape_msgs
#     visualization_msgs
#   DEPENDS
#     EIGEN3
#     OCTOMAP
#     console_bridge
#   )


include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIR} ${ASSIMP_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS})
include_directories(${rclcpp_INCLUDE_DIRS}
                    ${rmw_implementation_INCLUDE_DIRS}
                    ${console_bridge_INCLUDE_DIRS})
# include_directories(${catkin_INCLUDE_DIRS} ${console_bridge_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/bodies.cpp
  src/body_operations.cpp
  src/mesh_operations.cpp
  src/shape_extents.cpp
  src/shape_operations.cpp
  src/shape_to_marker.cpp
  src/shapes.cpp
)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp shape_msgs
    visualization_msgs
    random_numbers
    eigen_stl_containers
    geometry_msgs
    resource_retriever)
# target_link_libraries(${PROJECT_NAME} ${ASSIMP_LIBRARIES} ${QHULL_LIBRARIES} ${catkin_LIBRARIES} ${console_bridge_LIBRARIES} ${Boost_LIBRARIES})

# if(CATKIN_ENABLE_TESTING)
#   # Unit tests
#   add_subdirectory(test)
# endif()

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME})

ament_package()
