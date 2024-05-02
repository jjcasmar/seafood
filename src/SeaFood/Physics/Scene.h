#ifndef D93425D4_9F5D_4082_9243_534BF94798DA
#define D93425D4_9F5D_4082_9243_534BF94798DA

#include <Eigen/Dense>
#include "SeaFood/Core/Types.h"
#include "SeaFood/Physics/seaphysics_export.h"

#include <SeaFood/Physics/Boid.h>

#include <embree3/rtcore.h>
#include <embree3/rtcore_device.h>

namespace seafood::physics
{

// A basic mesh to represent obstacles
struct Mesh {
    std::vector<Eigen::Matrix<Real, 3, 1>> points;  // A vector, because embree later expects XYZXYZXYZ...
    std::vector<std::array<int, 3>> faces;
};

// A scene is made of a boid and a list of obstacles we want to avoid
class SEAPHYSICS_EXPORT Scene
{
public:
    Scene() = default;
    ~Scene();
    // Builds the underlying structures to support collision detection
    void commit();

    Boid &boid();
    std::vector<Mesh> &meshes();

    const Boid &boid() const;
    const std::vector<Mesh> &meshes() const;

    const RTCScene &bvh() const;

private:
    Boid m_boid{};
    std::vector<Mesh> m_meshes;

    RTCDevice m_device{};
    RTCScene m_scene{};
};

}  // namespace seafood::physics

#endif /* D93425D4_9F5D_4082_9243_534BF94798DA */
