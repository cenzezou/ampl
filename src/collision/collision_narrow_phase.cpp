
#include <cmath>
#include <iostream>
#include <ampl/geometry.hpp>
#include <ampl/collision.hpp>
#include "../algo/simd_sse.hpp"
namespace ampl
{

template void collision_initialize_object<512>( VSphG8f<512> &, const float *xyzr, uint32_t nb_sph,
                                                const uint32_t *offset, uint32_t nb_offset );

template <int A>
void collision_initialize_object( VSphG8f<A> &spheres, const float *xyzr, uint32_t nb_sph, const uint32_t *offset,
                                  uint32_t nb_offset )
{
  constexpr float RADIUS_NEG_INF = -1E4;
  spheres.nb_offset              = nb_offset;
  spheres.nb_sph                 = 0;
  float *x                       = spheres.x;
  float *y                       = spheres.y;
  float *z                       = spheres.z;
  float *r                       = spheres.r;
  uint32_t offset_cur            = 0;
  for ( uint32_t i = 0; i < nb_offset; i++ )
  {
    const uint32_t offset_true = offset[ i ];
    const uint32_t offset_pad  = offset[ i ] % 4 == 0 ? offset[ i ] : offset[ i ] + ( 4 - ( offset[ i ] % 4 ) );

    for ( size_t k = 0; k < offset_true; k++ )
    {
      x[ k ] = xyzr[ 4 * offset_cur + 4 * k + 0 ];
      y[ k ] = xyzr[ 4 * offset_cur + 4 * k + 1 ];
      z[ k ] = xyzr[ 4 * offset_cur + 4 * k + 2 ];
      r[ k ] = xyzr[ 4 * offset_cur + 4 * k + 3 ];
    }
    for ( size_t k = offset_true; k < offset_pad; k++ )
    {
      x[ k ] = xyzr[ 4 * offset_cur + 4 * ( offset_true - 1 ) + 0 ];
      y[ k ] = xyzr[ 4 * offset_cur + 4 * ( offset_true - 1 ) + 1 ];
      z[ k ] = xyzr[ 4 * offset_cur + 4 * ( offset_true - 1 ) + 2 ];
      r[ k ] = RADIUS_NEG_INF;
    }

    offset_cur += offset_true;

    spheres.offset[ i ] = i == 0 ? offset_pad : spheres.offset[ i - 1 ] + offset_pad;
    spheres.nb_sph      = spheres.offset[ i ];
    x += offset_pad;
    y += offset_pad;
    z += offset_pad;
    r += offset_pad;
  }
  printf_debug( "nb_offset = %u\n", spheres.nb_offset );
  printf_debug( "   nb_sph = %u\n", spheres.nb_sph );

#if AMPL_DEBUG == 1
  for ( uint32_t i = 0; i < spheres.nb_offset; i++ )
    for ( uint32_t k = i == 0 ? 0 : spheres.offset[ i ]; k < spheres.offset[ i + 1 ]; k++ )
      printf_debug( "     xyzr%u = %+01.5f, %+01.5f, %+01.5f, %+01.5f\n", spheres.offset[ i ], spheres.x[ k ],
                    spheres.y[ k ], spheres.z[ k ], spheres.r[ k ] );
#endif
};

bool collision_df_vsph( const DistanceField &vsph, const float *x, const float *y, const float *z, const float *r,
                        const uint32_t nb_sph, float d_safe )
{
  return false;
};

}  // namespace ampl