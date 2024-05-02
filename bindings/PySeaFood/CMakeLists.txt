find_package(pybind11)

set(SOURCES
    PySeaFood.cpp
)

set(HEADERS
)

pybind11_add_module(PySeaFood ${SOURCES} ${HEADERS})
target_link_libraries(PySeaFood PUBLIC SeaPhysics pybind11::module pybind11::headers)