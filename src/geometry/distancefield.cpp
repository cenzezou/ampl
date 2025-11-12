#include "edt.hpp"
#include <cmath>
#include <iostream>
#include <ampl/geometry.hpp>

namespace ampl
{
template void distancefield_xyz2occ<double, unsigned char>( const double *XYZ, uint32_t num_xyz, unsigned char *occ,
                                                            uint32_t ni, uint32_t nj, uint32_t nk, double min_x,
                                                            double min_y, double min_z, double d_vol );
template void distancefield_xyz2occ<float, unsigned char>( const float *XYZ, uint32_t num_xyz, unsigned char *occ,
                                                           uint32_t ni, uint32_t nj, uint32_t nk, float min_x,
                                                           float min_y, float min_z, float d_vol );
template void distancefield_xyz2occ<double, unsigned char>( const double *X, const double *Y, const double *Z,
                                                            uint32_t num_xyz, unsigned char *occ, uint32_t ni,
                                                            uint32_t nj, uint32_t nk, double min_x, double min_y,
                                                            double min_z, double d_vol );
template void distancefield_xyz2occ<float, unsigned char>( const float *X, const float *Y, const float *Z,
                                                           uint32_t num_xyz, unsigned char *occ, uint32_t ni,
                                                           uint32_t nj, uint32_t nk, float min_x, float min_y,
                                                           float min_z, float d_vol );

// template void distancefield_occ2edf<double, unsigned char>( const unsigned char *occ, double *edf, uint32_t ni,
//                                                             uint32_t nj, uint32_t nk, double d_vol,
//                                                             uint32_t nb_worker );
template void distancefield_occ2edf<float, unsigned char>( const unsigned char *occ, float *edf, uint32_t ni,
                                                           uint32_t nj, uint32_t nk, float d_vol, uint32_t nb_worker );

template <typename REAL, typename UINT>
void distancefield_xyz2occ( const REAL *X, const REAL *Y, const REAL *Z, uint32_t num_xyz, UINT *occ, uint32_t ni,
                            uint32_t nj, uint32_t nk, REAL min_x, REAL min_y, REAL min_z, REAL dx )
{
  REAL scale    = (REAL)1 / dx;
  const int nx  = ni;
  const int ny  = nj;
  const int nz  = nk;
  const int nij = ni * nj;
  // const REAL *x_this       = X;
  // const REAL *y_this       = Y;
  // const REAL *z_this       = Z;

  for ( int i = 0; i < nij * nz; i++ ) occ[ i ] = (UINT)1;

  for ( int i = 0; i < (int)num_xyz; i++ )
  {
    int x = std::round( ( X[ i ] - min_x ) * scale );
    int y = std::round( ( Y[ i ] - min_y ) * scale );
    int z = std::round( ( Z[ i ] - min_z ) * scale );
    if ( x < 0 || x >= nx || y < 0 || y >= ny || z < 0 || z >= nz ) continue;
    occ[ z * nij + y * ni + x ] = 0;
  }

  return;
};

template <typename REAL, typename UINT>
void distancefield_xyz2occ( const REAL *XYZ, uint32_t num_xyz, UINT *occ, uint32_t ni, uint32_t nj, uint32_t nk,
                            REAL min_x, REAL min_y, REAL min_z, REAL dx )
{
  REAL scale    = (REAL)1 / dx;
  const int nx  = ni;
  const int ny  = nj;
  const int nz  = nk;
  const int nij = ni * nj;
  //
  // const REAL *y_this       = Y;
  // const REAL *z_this       = Z;

  for ( int i = 0; i < nij * nz; i++ ) occ[ i ] = (UINT)1;

  const REAL *ptr_xyz = XYZ;
  for ( int i = 0; i < (int)num_xyz; i++ )
  {
    int x = std::round( ( ptr_xyz[ 0 ] - min_x ) * scale );
    int y = std::round( ( ptr_xyz[ 1 ] - min_y ) * scale );
    int z = std::round( ( ptr_xyz[ 2 ] - min_z ) * scale );
    if ( x < 0 || x >= nx || y < 0 || y >= ny || z < 0 || z >= nz )
    {
    }
    else
      occ[ z * nij + y * ni + x ] = 0;

    ptr_xyz += 3;
  }

  return;
};

template <typename REAL, typename UINT>
void distancefield_occ2edf( const UINT *occ, REAL *edf, uint32_t ni, uint32_t nj, uint32_t nk, REAL dx,
                            uint32_t nb_worker )
{
  edt::binary_edt<UINT>( const_cast<UINT *>( occ ), ni, nj, nk, dx, dx, dx, false, nb_worker, edf );
  return;
};

}  // namespace ampl