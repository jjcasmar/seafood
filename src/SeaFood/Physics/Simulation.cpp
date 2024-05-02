#include "Simulation.h"

#include <nanoflann.hpp>

#include <SeaFood/Physics/Boid.h>
#include <SeaFood/Core/Types.h>

#include <embree3/rtcore_common.h>
#include <embree3/rtcore_ray.h>

#include <spdlog/spdlog.h>

namespace seafood::physics
{

void Simulation::step(Scene &scene, seafood::Real h) const
{
    auto &boid{scene.boid()};

    // Compute a KDTree to compute neighbors neighbors fast
    auto kdTree = nanoflann::KDTreeEigenMatrixAdaptor<Boid::PositionT>(3, std::cref(boid.x));

    // We want to compute the velocities for moving the particles
    // For each particle, we have several contributions:
    // - Cohesion: We try to move particles to the centroid of the neighbor particle it sees
    // - Separation: Tries to go away from too close particles
    // - Steering: Tries to have the same direction as nearby particles
    // - Goal: Tries to go towards a goal
    // - Obstacles: Tries to avoid obstacles

    // We limite the amount of neighbor the particles can keep track of
    // Therefore, we can use std::array to store closest neighbors
    constexpr auto PerceptionCapacity = 48;
    std::array<Eigen::Index, PerceptionCapacity> neighborIndices{};
    std::array<Real, PerceptionCapacity> distances{};

    const auto nbParticles{boid.x.rows()};
    for (int pId = 0; pId < nbParticles; ++pId) {
        const Eigen::Matrix<Real, 3, 1> x = boid.x.row(pId);
        const Eigen::Matrix<Real, 3, 1> v = boid.v.row(pId);

        nanoflann::KNNResultSet<Real, Eigen::Index> resultSet(PerceptionCapacity);

        resultSet.init(neighborIndices.data(), distances.data());
        kdTree.index_->findNeighbors(resultSet, x.data());

        auto meanVelocityDir = Eigen::Matrix<Real, 3, 1>::Zero().eval();
        auto avoidCrowdDir = Eigen::Matrix<Real, 3, 1>::Zero().eval();
        auto com = Eigen::Matrix<Real, 3, 1>::Zero().eval();

        float comWeight = 0;

        for (int i = 0; i < PerceptionCapacity; ++i) {
            const auto neighborId = neighborIndices[i];
            const auto distance = distances[i];

            const Eigen::Matrix<Real, 3, 1> neighborPosition = boid.x.row(neighborId);
            const auto isAhead = (neighborPosition - x).dot(v) > 0;

            // We only care abut particles infront of us. We can't see behind...
            if (distance < perceptionRadius && isAhead) {
                const auto coeff{1.0F / std::exp(-distance)};
                comWeight += coeff;
                // Get the mean velocity of the local flockmate
                meanVelocityDir += coeff * boid.v.row(neighborId).transpose().normalized().eval();

                // Compute center of mass contribution
                com += coeff * neighborPosition;

                // If the neighbor is too close, go away
                // This assumes the crowded distance is lower than the perception distance. Which makes sense
                if (distance < crowdedRadius) {
                    avoidCrowdDir -= (neighborPosition - x).normalized();
                }
            }
        }

        // Avoid obstacles
        const auto avoidDir{avoidObstaclesDir(scene, pId)};

        // Now we just have to weight each velocity to compute a new velocity
        // This actually might make things complex, as we are modifying velocities that are going to be used by other
        // particles to compute their steering velocity
        boid.v.row(pId).transpose() = v  //
                                      + avoidObstacleScale * avoidDir;
        if (comWeight > 0) {
            boid.v.row(pId).transpose() +=                                      //
                crowdedVelocityScale * avoidCrowdDir                            //
                + cohesionVelocityScale * ((com / comWeight - x).normalized())  //
                + steeringScale * meanVelocityDir / comWeight;
        }

        // Clamp velocity
        const auto n = boid.v.row(pId).norm();
        boid.v.row(pId) = boid.v.row(pId) / n * maxVelocity;
    }

    boid.x += h * boid.v;
}

Eigen::Matrix<Real, 3, 1> Simulation::avoidObstaclesDir(const Scene &scene, int pId) const
{
    Eigen::Matrix<Real, 3, 1> x = scene.boid().x.row(pId);
    Eigen::Matrix<Real, 3, 1> v = scene.boid().v.row(pId).normalized();

    // We need to shoot a ray to check if we are going to collide
    RTCRayHit rayhit;
    rayhit.ray.org_x = x.x();
    rayhit.ray.org_y = x.y();
    rayhit.ray.org_z = x.z();

    rayhit.ray.dir_x = v.x();
    rayhit.ray.dir_y = v.y();
    rayhit.ray.dir_z = v.z();

    rayhit.ray.tnear = 0;
    rayhit.ray.tfar = perceptionRadius;  // We can only see up to the percention radius
    rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;

    RTCIntersectContext context;
    rtcInitIntersectContext(&context);
    rtcIntersect1(scene.bvh(), &context, &rayhit);

    // If we hit something
    const auto hitGeomId = rayhit.hit.geomID;
    if (hitGeomId != RTC_INVALID_GEOMETRY_ID) {
        const auto primId = rayhit.hit.primID;

        // Compute normal of the hit
        const auto &hitMesh = scene.meshes()[hitGeomId];
        const auto p0 = hitMesh.points[hitMesh.faces[primId][0]];
        const auto p1 = hitMesh.points[hitMesh.faces[primId][1]];
        const auto p2 = hitMesh.points[hitMesh.faces[primId][2]];

        auto n = ((p1 - p0).cross(p2 - p0).normalized()).eval();

        // We dont care which side of the triangle we are, project the normal on the same direction as the velocity
        if (v.dot(n) < 0) {
            n *= -1;
        }

        // And now, remove the boid velocity on the normal direction
        return (v - n * v.dot(n)).normalized();
    }
    return Eigen::Matrix<Real, 3, 1>::Zero();
}

}  // namespace seafood::physics