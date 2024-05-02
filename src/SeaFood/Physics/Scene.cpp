#include "Scene.h"
#include <embree3/rtcore_buffer.h>
#include <embree3/rtcore_common.h>
#include <embree3/rtcore_geometry.h>
#include <embree3/rtcore_scene.h>

namespace seafood::physics
{

Scene::~Scene()
{
    rtcReleaseScene(m_scene);
    rtcReleaseDevice(m_device);
}

Boid &Scene::boid()
{
    return m_boid;
}

const Boid &Scene::boid() const
{
    return m_boid;
}

std::vector<Mesh> &Scene::meshes()
{
    return m_meshes;
}

const std::vector<Mesh> &Scene::meshes() const
{
    return m_meshes;
}

void Scene::commit()
{
    m_device = rtcNewDevice(nullptr);
    m_scene = rtcNewScene(m_device);

    for (const auto &mesh : m_meshes) {
        auto *geometry{rtcNewGeometry(m_device, RTC_GEOMETRY_TYPE_TRIANGLE)};
        auto *vertexBuffer{static_cast<float *>(rtcSetNewGeometryBuffer(geometry,  //
                                                                        RTC_BUFFER_TYPE_VERTEX,
                                                                        0,
                                                                        RTC_FORMAT_FLOAT3,
                                                                        3 * sizeof(Real),
                                                                        mesh.points.size()))};

        // Assign vertices to embree buffer
        Eigen::Matrix<Real, Eigen::Dynamic, 3, Eigen::RowMajor>::MapType(
            vertexBuffer, static_cast<Eigen::Index>(mesh.points.size()), 3) =
            Eigen::Matrix<Real, Eigen::Dynamic, 3, Eigen::RowMajor>::ConstMapType(
                mesh.points.data()->data(), static_cast<Eigen::Index>(mesh.points.size()), 3);

        auto *indexBuffer{static_cast<int *>(rtcSetNewGeometryBuffer(geometry,  //
                                                                     RTC_BUFFER_TYPE_INDEX,
                                                                     0,
                                                                     RTC_FORMAT_UINT3,
                                                                     3 * sizeof(unsigned int),
                                                                     mesh.faces.size()))};

        for (int i = 0; i < mesh.faces.size(); ++i) {
            indexBuffer[3 * i + 0] = mesh.faces[i][0];
            indexBuffer[3 * i + 1] = mesh.faces[i][1];
            indexBuffer[3 * i + 2] = mesh.faces[i][2];
        }
        // And assign indices
        // Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor>::MapType(
        //     indexBuffer, static_cast<Eigen::Index>(mesh.faces.size()), 3) =
        //     Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor>::ConstMapType(
        //         mesh.faces.data()->data(), static_cast<Eigen::Index>(mesh.faces.size()), 3);

        rtcCommitGeometry(geometry);
        rtcAttachGeometry(m_scene, geometry);

        // Object in embree are refcounted. We need to release our "copy"
        rtcReleaseGeometry(geometry);
    }

    rtcCommitScene(m_scene);
}

const RTCScene &Scene::bvh() const
{
    return m_scene;
}
}  // namespace seafood::physics