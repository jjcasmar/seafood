#ifndef AC15AD06_F625_48CB_8510_231180996559
#define AC15AD06_F625_48CB_8510_231180996559

#include <Eigen/Sparse>

#include <SeaFood/Core/Types.h>

#include <SeaFood/Physics/seaphysics_export.h>

namespace seafood::physics
{

struct SEAPHYSICS_EXPORT Boid {
    using WorldScalar = seafood::Real;

    // For position, we have a bunch of 3D positions in space
    // We organize them as a Matrix Nx3, so all the xs, ys and zs are adjacent
    // In memory layout we have something like
    // XXXXXXXXXYYYYYYYYYZZZZZZZZ
    // which will help us using vectorized computations
    using PositionT = Eigen::Matrix<seafood::Real, Eigen::Dynamic, 3>;
    using VelocityT = Eigen::Matrix<seafood::Real, Eigen::Dynamic, 3>;

    PositionT x;
    VelocityT v;
};

}  // namespace seafood::physics

#endif /* AC15AD06_F625_48CB_8510_231180996559 */
