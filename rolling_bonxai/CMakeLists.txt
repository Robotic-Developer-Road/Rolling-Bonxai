cmake_minimum_required(VERSION 3.8)
project(rolling_bonxai)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================================================================
# Find dependencies
# ============================================================================

find_package(ament_cmake REQUIRED)
find_package(bonxai_core REQUIRED)
find_package(bonxai_map REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# ============================================================================
# Create the library
# ============================================================================

add_library(${PROJECT_NAME} SHARED
  src/common.cpp
  src/coordinate_system.cpp
  src/chunk_storage.cpp
  src/chunk_loading_policies.cpp
  src/file_storage_backend.cpp
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
    yaml-cpp
)

# Export dependencies for downstream packages
ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
    Eigen3
)

# ============================================================================
# Install library
# ============================================================================

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

# ============================================================================
# Testing
# ============================================================================

if(BUILD_TESTING)
  # turn off linting because our codestyle is fucked
  # find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  find_package(bonxai_core REQUIRED)
  find_package(bonxai_map REQUIRED)

  add_library(${PROJECT_NAME}_testlib STATIC
    src/common.cpp
    src/coordinate_system.cpp
    src/chunk_storage.cpp
    src/chunk_loading_policies.cpp
    src/file_storage_backend.cpp
  )

  target_include_directories(${PROJECT_NAME}_testlib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(${PROJECT_NAME}_testlib PUBLIC
    bonxai_core::bonxai_core
    bonxai_map::bonxai_map
    yaml-cpp
  )

  ament_target_dependencies(${PROJECT_NAME}_testlib PUBLIC
    Eigen3
  )

  ## Common tests
  ament_add_gtest(test_common
    test/test_common.cpp
  )

  target_include_directories(test_common PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(test_common
    ${PROJECT_NAME}_testlib  # Link against the static library
  )

  ament_target_dependencies(test_common
    Eigen3
    bonxai_core
    bonxai_map
  )

  ## Coordinate System tests
  ament_add_gtest(test_coordinate_system
    test/test_coordinate_system.cpp
  )

  target_include_directories(test_coordinate_system PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(test_coordinate_system
    ${PROJECT_NAME}_testlib  # Link against the static library
  )

  ament_target_dependencies(test_coordinate_system
    Eigen3
    bonxai_core
    bonxai_map
  )

    ## File Storage Backend tests
  ament_add_gtest(test_file_storage_backend
    test/test_file_storage_backend.cpp
  )

  target_include_directories(test_file_storage_backend PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(test_file_storage_backend
    ${PROJECT_NAME}_testlib  # Link against the static library
  )

  ament_target_dependencies(test_file_storage_backend
    Eigen3
    bonxai_core
    bonxai_map
  )

  ## Chunk Storage tests
  ament_add_gtest(test_chunk_storage
    test/test_chunk_storage.cpp
  )

  target_include_directories(test_chunk_storage PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(test_chunk_storage
    ${PROJECT_NAME}_testlib  # Link against the static library
  )

  ament_target_dependencies(test_chunk_storage
    Eigen3
    bonxai_core
    bonxai_map
  )

  ## Chunk Loading Policy
  ament_add_gtest(test_chunk_loading_policy
    test/test_chunk_loading_policy.cpp
  )

  target_include_directories(test_chunk_loading_policy PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(test_chunk_loading_policy
    ${PROJECT_NAME}_testlib  # Link against the static library
  )

  ament_target_dependencies(test_chunk_loading_policy
    Eigen3
    bonxai_core
    bonxai_map
  )

endif()

# ============================================================================
# Optional: Build Benchmarks
# ============================================================================

option(BUILD_BENCHMARKS "Build benchmark executables" OFF)

if(BUILD_BENCHMARKS)
  message(STATUS "Building benchmarks (BUILD_BENCHMARKS=ON)")
  add_subdirectory(benchmark)
else()
  message(STATUS "Skipping benchmarks (BUILD_BENCHMARKS=OFF)")
endif()

# ============================================================================
# Package
# ============================================================================

ament_package()