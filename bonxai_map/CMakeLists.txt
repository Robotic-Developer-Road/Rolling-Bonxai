cmake_minimum_required(VERSION 3.8)
project(bonxai_map)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(bonxai_core REQUIRED)
find_package(Eigen3 REQUIRED)

# Create the library
add_library(${PROJECT_NAME} SHARED
  src/occupancy_map.cpp
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

# Link against bonxai_core (header-only)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    bonxai_core::bonxai_core
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
  Eigen3
)

# Export include directories
ament_export_include_directories(include)

# Testing
if(BUILD_TESTING)
  # turn off linting because our codestyle is fucked
  # find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  find_package(bonxai_core REQUIRED)

  add_library(${PROJECT_NAME}_testlib STATIC
    src/occupancy_map.cpp
  )
  target_include_directories(${PROJECT_NAME}_testlib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_testlib PUBLIC
    bonxai_core::bonxai_core
  )
  ament_target_dependencies(${PROJECT_NAME}_testlib PUBLIC
    Eigen3
  )

  ament_add_gtest(test_occupancy_map
    test/test_occupancy_map.cpp
  )

  target_include_directories(test_occupancy_map PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(test_occupancy_map
    ${PROJECT_NAME}_testlib  # Link against the static library
  )

  ament_target_dependencies(test_occupancy_map
    Eigen3
    bonxai_core
  )


endif()

ament_package()