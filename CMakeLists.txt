cmake_minimum_required(VERSION 3.14)
project(MyProject CXX)

# Enable testing
enable_testing()

# Use C++17 or newer
set(CMAKE_CXX_STANDARD 17)

# Grab eigen
include(FetchContent)
set(INSTALL_GTEST OFF)
set(BUILD_GTEST ON)
set(BUILD_GMOCK OFF)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF)  # ⬅️ disables GoogleTest's own test suite

# Add source directories
add_subdirectory(src/unicycle)
target_include_directories(unicycle PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Add test directory
add_subdirectory(test)
