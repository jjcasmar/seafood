#include <pybind11/detail/common.h>
#include <pybind11/pybind11.h>

#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <SeaFood/Core/Types.h>
#include <SeaFood/Physics/Boid.h>
#include <SeaFood/Physics/Scene.h>
#include <SeaFood/Physics/Simulation.h>

PYBIND11_MODULE(PySeaFood, m)
{
    m.attr("__version__") = "dev";

    pybind11::class_<seafood::physics::Boid>(m, "Boid")
        .def(pybind11::init<>())
        .def_readwrite("x", &seafood::physics::Boid::x)
        .def_readwrite("v", &seafood::physics::Boid::v);

    pybind11::class_<seafood::physics::Mesh>(m, "Mesh")
        .def(pybind11::init<>())
        .def_readwrite("points", &seafood::physics::Mesh::points)
        .def_readwrite("faces", &seafood::physics::Mesh::faces);

    pybind11::class_<seafood::physics::Scene>(m, "Scene")
        .def(pybind11::init<>())
        .def_property(
            "boid",
            [](const seafood::physics::Scene &scene) { return scene.boid(); },
            [](seafood::physics::Scene &scene, const seafood::physics::Boid &boid) { scene.boid() = boid; })
        .def_property(
            "meshes",
            [](const seafood::physics::Scene &scene) { return scene.meshes(); },
            [](seafood::physics::Scene &scene, const std::vector<seafood::physics::Mesh> &meshes) {
                scene.meshes() = meshes;
            })
        .def("commit", &seafood::physics::Scene::commit);

    pybind11::class_<seafood::physics::Simulation>(m, "Simulation")
        .def(pybind11::init<>())
        .def_readwrite("perceptionRadius", &seafood::physics::Simulation::perceptionRadius)
        .def_readwrite("crowdedRadius", &seafood::physics::Simulation::crowdedRadius)
        .def_readwrite("maxVelocity", &seafood::physics::Simulation::maxVelocity)
        .def_readwrite("inertialScale", &seafood::physics::Simulation::inertiaVelocityScale)
        .def_readwrite("cohesionVelocityScale", &seafood::physics::Simulation::cohesionVelocityScale)
        .def_readwrite("crowdedVelocityScale", &seafood::physics::Simulation::crowdedVelocityScale)
        .def_readwrite("steeringScale", &seafood::physics::Simulation::steeringScale)
        .def_readwrite("avoidObstacleScale", &seafood::physics::Simulation::avoidObstacleScale)
        .def_readwrite("randomization", &seafood::physics::Simulation::randomizationScale)
        .def("step", &seafood::physics::Simulation::step);
}