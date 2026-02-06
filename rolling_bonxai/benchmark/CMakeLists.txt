# Benchmark CMakeLists.txt for Rolling Bonxai Chunk Storage
cmake_minimum_required(VERSION 3.14)

# Note: This assumes it's being included from parent CMakeLists.txt
# If building standalone, uncomment the following:
# project(rolling_bonxai_benchmarks LANGUAGES CXX)
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

message(STATUS "Configuring Rolling Bonxai benchmarks...")

# ============================================================================
# Fetch nanobench
# ============================================================================
include(FetchContent)

FetchContent_Declare(
    nanobench
    GIT_REPOSITORY https://github.com/martinus/nanobench.git
    GIT_TAG v4.1.0
    GIT_SHALLOW TRUE
)

# Make nanobench available
FetchContent_MakeAvailable(nanobench)

message(STATUS "nanobench fetched successfully")

# ============================================================================
# Benchmark Executable: chunk_storage_benchmark
# ============================================================================

add_executable(chunk_storage_benchmark
    src/chunk_storage_benchmark.cpp
)

# Include directories
target_include_directories(chunk_storage_benchmark
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../include  # Parent project includes
)

# Link libraries
target_link_libraries(chunk_storage_benchmark
    PRIVATE
        nanobench
        rolling_bonxai  # Main library (chunk_storage, common, etc.)
)

# Compiler options for benchmarks (optimizations)
target_compile_options(chunk_storage_benchmark
    PRIVATE
        -O3                    # Maximum optimization
        -march=native          # Use CPU-specific optimizations
        -DNDEBUG              # Disable assertions
        $<$<CXX_COMPILER_ID:GNU>:-fno-omit-frame-pointer>
        $<$<CXX_COMPILER_ID:Clang>:-fno-omit-frame-pointer>
)

# Set output directory
set_target_properties(chunk_storage_benchmark
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/benchmarks
)

message(STATUS "Benchmark executable: chunk_storage_benchmark")

# ============================================================================
# Installation (optional)
# ============================================================================

# Install benchmark executable
install(TARGETS chunk_storage_benchmark
    RUNTIME DESTINATION lib/${PROJECT_NAME}/benchmarks
)

message(STATUS "Benchmark configuration complete")