#ifndef AMPL_COLLISION_HPP
#define AMPL_COLLISION_HPP

#include <ampl/common.hpp>

namespace ampl
{
namespace soa
{
template <typename REAL, int MAX_SPHERE_SIZE, int MAX_OFFSET_SIZE>
struct alignas( 16 ) SphereGroup
{
  alignas( 16 ) REAL x[ MAX_SPHERE_SIZE ];
  alignas( 16 ) REAL y[ MAX_SPHERE_SIZE ];
  alignas( 16 ) REAL z[ MAX_SPHERE_SIZE ];
  alignas( 16 ) REAL r[ MAX_SPHERE_SIZE ];
  uint32_t offset[ MAX_OFFSET_SIZE ];
  uint32_t num_offset                     = 0;
  constexpr static uint32_t nb_max_sph    = MAX_SPHERE_SIZE;
  constexpr static uint32_t nb_max_offset = MAX_OFFSET_SIZE;
};

using VSphG256f  = SphereGroup<float, 256, 8>;
using VSphG512f  = SphereGroup<float, 512, 8>;
using VSphG1024f = SphereGroup<float, 1024, 8>;

}  // namespace soa
}  // namespace ampl

#endif