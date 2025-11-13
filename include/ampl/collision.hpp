#ifndef AMPL_COLLISION_HPP
#define AMPL_COLLISION_HPP

#include <ampl/common.hpp>

namespace ampl
{
namespace soa
{
template <int MAX_SPHERE_SIZE, int MAX_OFFSET_SIZE>
struct SphereGroupf
{
  float x[ MAX_SPHERE_SIZE ];
  float y[ MAX_SPHERE_SIZE ];
  float z[ MAX_SPHERE_SIZE ];
  float r[ MAX_SPHERE_SIZE ];
  uint32_t offset[ MAX_OFFSET_SIZE ];
  uint32_t num_offset                     = 0;
  constexpr static uint32_t nb_max_sph    = MAX_SPHERE_SIZE;
  constexpr static uint32_t nb_max_offset = MAX_OFFSET_SIZE;
  SphereGroupf(){};
};

template <typename VSphGf>
void collision_initialize_object( VSphGf &, const float *xyzr, uint32_t nb_sph, const uint32_t *offset,
                                  uint32_t nb_offset );

typedef SphereGroupf<512, 8> VSphG512f;

}  // namespace soa
}  // namespace ampl

#endif