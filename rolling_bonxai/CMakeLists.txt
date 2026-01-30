cmake_minimum_required(VERSION 3.8)
project(rolling_bonxai)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(bonxai_core REQUIRED)
find_package(bonxai_map REQUIRED)
find_package(Eigen3 REQUIRED)

# Create the library
add_library(${PROJECT_NAME} SHARED
  src/coordinate_system.cpp
)

# Set C++17 standard
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# Include directories
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)

# Link against bonxai_core and bonxai_map (header-only)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    bonxai_core::bonxai_core
    bonxai_map::bonxai_map
)

# Export dependencies for downstream packages
ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
    Eigen3
)

# Install library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

# Export targets
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

# Export dependencies
ament_export_dependencies(
  bonxai_core
  bonxai_map
  Eigen3
)

# Export include directories
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
