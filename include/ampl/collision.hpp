#ifndef AMPL_COLLISION_HPP
#define AMPL_COLLISION_HPP

#include <ampl/common.hpp>
#include <vector>
namespace ampl
{
namespace soa
{

template <int MAX_SPHERE_SIZE, int MAX_OFFSET_SIZE>
struct SphereGroupf
{
  float x[ MAX_SPHERE_SIZE ] AMPL_ALIGNED( 16 );
  float y[ MAX_SPHERE_SIZE ] AMPL_ALIGNED( 16 );
  float z[ MAX_SPHERE_SIZE ] AMPL_ALIGNED( 16 );
  float r[ MAX_SPHERE_SIZE ] AMPL_ALIGNED( 16 );
  uint32_t offset[ MAX_OFFSET_SIZE ];
  uint32_t nb_offset                      = 0;
  uint32_t nb_sph                         = 0;
  constexpr static uint32_t nb_max_sph    = MAX_SPHERE_SIZE;
  constexpr static uint32_t nb_max_offset = MAX_OFFSET_SIZE;
};
}  // namespace soa

template <int A>
using VSphG8f = soa::SphereGroupf<A, 8>;

struct DistanceField
{
  std::vector<float> _data;
  uint32_t shape[ 3 ];
  float origin[ 3 ];
  float side;
  void set_shape( uint32_t nx, uint32_t ny, uint32_t nz )
  {
    shape[ 0 ] = nx;
    shape[ 1 ] = ny;
    shape[ 2 ] = nz;
    _data.resize( nx * ny * nz );
  };
  void set_origin( float ox, float oy, float oz )
  {
    origin[ 0 ] = ox;
    origin[ 1 ] = oy;
    origin[ 2 ] = oz;
  }
  void set_side( float s ) { side = s; }
  void clear()
  {
    side        = 0;
    origin[ 0 ] = 0;
    origin[ 1 ] = 0;
    origin[ 2 ] = 0;
    shape[ 0 ]  = 0;
    shape[ 1 ]  = 0;
    shape[ 2 ]  = 0;
    _data.clear();
  }
  const float *data() const { return _data.data(); };
};

template <int A>
void collision_initialize_object( VSphG8f<A> &vsph, const float *xyzr, uint32_t nb_sph, const uint32_t *offset,
                                  uint32_t nb_offset );

bool collision_df_vsph( const DistanceField &vsph, const float *x, const float *y, const float *z, const float *r,
                        const uint32_t nb_sph, float d_safe );

}  // namespace ampl

#endif