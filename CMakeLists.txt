cmake_minimum_required(VERSION 3.23)
project(SeaFood LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Bring packages that are not available in Conan
# Use with care
add_subdirectory(3rdparty)

find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(nanoflann REQUIRED)
find_package(embree REQUIRED)

set(BUILD_SHARED_LIBS True)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN YES)

include(GenerateExportHeader)
include(GNUInstallDirs)

# Set output directories for libraries and binaries.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)


add_subdirectory(src)
add_subdirectory(bindings)


#if(BUILD_TESTING)
#  find_package(GTest REQUIRED)
#  enable_testing()
#  message(STATUS "Generating tests")
#  add_subdirectory(test)
#endif()
