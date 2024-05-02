#ifndef VELO_PHYSICS_NBEULER_H
#define VELO_PHYSICS_NBEULER_H

#include <nanoflann.hpp>

#include <SeaFood/Core/Types.h>

#include <SeaFood/Physics/Boid.h>
#include <SeaFood/Physics/Scene.h>

#include <SeaFood/Physics/seaphysics_export.h>

namespace seafood::physics
{

struct SEAPHYSICS_EXPORT Simulation {
    void step(Scene &scene, seafood::Real h) const;

    Real perceptionRadius = Real{1.0};
    Real crowdedRadius = Real{0.1};
    Real maxVelocity = Real{1.0};

    Real inertiaVelocityScale{0};
    Real cohesionVelocityScale{0};
    Real crowdedVelocityScale{0};
    Real steeringScale{0};
    Real randomizationScale{0};
    Real avoidObstacleScale{0};

private:
    SEAPHYSICS_NO_EXPORT Eigen::Matrix<Real, 3, 1> avoidObstaclesDir(const Scene &scene, int pId) const;
};

}  // namespace seafood::physics

#endif  //  VELO_PHYSICS_NBEULER_H
