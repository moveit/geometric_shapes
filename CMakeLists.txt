cmake_minimum_required(VERSION 3.5)
project(geometric_shapes)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Set compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(PROJECT_COMPILE_OPTIONS
    -Wall
    -Wextra
    -Wwrite-strings
    -Wunreachable-code
    -Wpointer-arith
    -Wredundant-decls
    -Wno-unused-parameter
  )
endif()

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: Choosing 'Release' for maximum performance")
  set(CMAKE_BUILD_TYPE Release)
endif()

# TODO(henningkayser): Remove policy fix when assimp 5.1 is available
# Suppress policy warning in assimp (https://github.com/assimp/assimp/pull/2722)
set(CMAKE_POLICY_DEFAULT_CMP0012 NEW)
find_package(ASSIMP QUIET)
if(NOT ASSIMP_FOUND)
  find_package(PkgConfig REQUIRED)
  # assimp is required, so REQUIRE the second attempt
  pkg_check_modules(ASSIMP_PC REQUIRED assimp)
  set(ASSIMP_INCLUDE_DIRS ${ASSIMP_PC_INCLUDE_DIRS})
endif()

# find *absolute* paths to ASSIMP_LIBRARIES
# Both, pkg-config and assimp's cmake-config don't provide an absolute library path.
# For, pkg-config the path is in ASSIMP_PC_LIBRARY_DIRS, for cmake in ASSIMP_LIBRARY_DIRS.
find_library(ASSIMP_ABS_LIBRARIES NAMES ${ASSIMP_LIBRARIES} assimp HINTS ${ASSIMP_LIBRARY_DIRS} ${ASSIMP_PC_LIBRARY_DIRS})
set(ASSIMP_LIBRARIES "${ASSIMP_ABS_LIBRARIES}")

# These need to be in this order to find header files
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED filesystem)
find_package(console_bridge REQUIRED)
find_package(console_bridge_vendor REQUIRED)
find_package(eigen_stl_containers REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OCTOMAP REQUIRED)
find_package(QHULL REQUIRED)
find_package(random_numbers REQUIRED)
find_package(rclcpp REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(THIS_PACKAGE_EXPORT_DEPENDS
  eigen3_cmake_module
  Eigen3
  Boost
  console_bridge
  console_bridge_vendor
  eigen_stl_containers
  geometry_msgs
  OCTOMAP
  random_numbers
  rclcpp
  resource_retriever
  shape_msgs
  visualization_msgs
)

# Set VERSION from package.xml
ament_package_xml()

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/aabb.cpp
  src/bodies.cpp
  src/body_operations.cpp
  src/mesh_operations.cpp
  src/shape_extents.cpp
  src/shape_operations.cpp
  src/shape_to_marker.cpp
  src/shapes.cpp
)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION} WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
target_compile_options(${PROJECT_NAME} PRIVATE ${PROJECT_COMPILE_OPTIONS})
ament_target_dependencies(${PROJECT_NAME}
  ${THIS_PACKAGE_EXPORT_DEPENDS}
)
ament_target_dependencies(${PROJECT_NAME} SYSTEM
  # We don't export these dependencies because their cmake is broken
  ASSIMP
  QHULL
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Unit tests
  add_subdirectory(test)

  # ament_lint
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_EXPORT_DEPENDS})

ament_package()
