set(HEADERS
  Types.h
)

set(PUBLIC_HEADERS)

set(SOURCES)

add_library(SeaCore INTERFACE ${HEADERS} ${SOURCES} ${PUBLIC_HEADERS})

target_include_directories(SeaCore INTERFACE 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../..>
)

target_link_libraries(SeaCore INTERFACE Eigen3::Eigen)
