set(HEADERS
  Scene.h
  Boid.h
  Simulation.h
)

set(PUBLIC_HEADERS)

set(SOURCES
  Simulation.cpp
  Scene.cpp
)

add_library(SeaPhysics ${HEADERS} ${SOURCES} ${PUBLIC_HEADERS})

target_include_directories(SeaPhysics PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../..>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/../..>
)

target_link_libraries(SeaPhysics PUBLIC SeaCore Eigen3::Eigen nanoflann::nanoflann spdlog::spdlog embree)
generate_export_header(SeaPhysics)
