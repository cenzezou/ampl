
#include <Eigen/Dense>
#include <ampl/geometry.hpp>
#include <math.h>

#include <stdio.h>
namespace ampl
{

template double distance_point3_segment3<double>( const double *p, const double *v0, const double *v1,
                                                  double *v_closest, int dim, double *v_lambda );

template float distance_point3_segment3<float>( const float *p, const float *v0, const float *v1, float *v_closest,
                                                int dim, float *v_lambda );

template <typename FT>
FT distance_point3_segment3( const FT *p, const FT *v0, const FT *v1, FT *v_closest, int dim )
{
  return 0;
};

template <typename FT>
FT distance_point3_segment3( const FT *p, const FT *v0, const FT *v1, FT *v_closest, int dim, FT *v_lambda )
{
  return 0;
};

template double distance_point3_segment3<double>( const double *p, const double *v0, const double *v1,
                                                  double *v_closest, int dim );

template float distance_point3_segment3<float>( const float *p, const float *v0, const float *v1, float *v_closest,
                                                int dim );

template bool intersect_line3<float>( const float *, const float *, uint32_t, float * );

template bool intersect_line3<double>( const double *, const double *, uint32_t, double * );

template bool intersect_plane3<float>( const float *p0, const float *n0, const float *p1, const float *n1, float *o,
                                       float *t );
template bool intersect_plane3<double>( const double *p0, const double *n0, const double *p1, const double *n1,
                                        double *o, double *t );

template float distance_ray3_tri3<float>( const float *o, const float *d, const float *p0, const float *p1,
                                          const float *p2, const float &nearest );
template double distance_ray3_tri3<double>( const double *o, const double *d, const double *p0, const double *p1,
                                            const double *p2, const double &nearest );

template void bounding_aabb_tri3<float>( const float *v0, const float *v1, const float *v2, float *vmin, float *vmax,
                                         float epsilon );
template void bounding_aabb_tri3<double>( const double *v0, const double *v1, const double *v2, double *vmin,
                                          double *vmax, double epsilon );

template <typename REAL>
bool intersect_line3( const REAL *os, const REAL *ns, uint32_t nb_line, REAL *p );

template <typename REAL>
bool intersect_line3( const REAL *os, const REAL *ns, uint32_t num_lines, REAL *p )
{
  Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
  Eigen::Vector3d q = Eigen::Vector3d::Zero();

  Eigen::Matrix3d Qi;
  Eigen::Vector3d qi;

  for ( size_t i = 0; i < num_lines; i++ )
  {
    double nx = ns[ i * 3 + 0 ];
    double ny = ns[ i * 3 + 1 ];
    double nz = ns[ i * 3 + 2 ];
    double ox = os[ i * 3 + 0 ];
    double oy = os[ i * 3 + 1 ];
    double oz = os[ i * 3 + 2 ];
    Qi << 1 - 2 * std::pow( nx, 2 ) + std::pow( nx, 4 ) + std::pow( nx, 2 ) * std::pow( ny, 2 ) +
              std::pow( nx, 2 ) * std::pow( nz, 2 ),
        ( -4 * nx * ny + 2 * std::pow( nx, 3 ) * ny + 2 * nx * std::pow( ny, 3 ) + 2 * nx * ny * std::pow( nz, 2 ) ) /
            2.,
        ( -4 * nx * nz + 2 * std::pow( nx, 3 ) * nz + 2 * nx * std::pow( ny, 2 ) * nz + 2 * nx * std::pow( nz, 3 ) ) /
            2.,
        ( -4 * nx * ny + 2 * std::pow( nx, 3 ) * ny + 2 * nx * std::pow( ny, 3 ) + 2 * nx * ny * std::pow( nz, 2 ) ) /
            2.,
        1 - 2 * std::pow( ny, 2 ) + std::pow( nx, 2 ) * std::pow( ny, 2 ) + std::pow( ny, 4 ) +
            std::pow( ny, 2 ) * std::pow( nz, 2 ),
        ( -4 * ny * nz + 2 * std::pow( nx, 2 ) * ny * nz + 2 * std::pow( ny, 3 ) * nz + 2 * ny * std::pow( nz, 3 ) ) /
            2.,
        ( -4 * nx * nz + 2 * std::pow( nx, 3 ) * nz + 2 * nx * std::pow( ny, 2 ) * nz + 2 * nx * std::pow( nz, 3 ) ) /
            2.,
        ( -4 * ny * nz + 2 * std::pow( nx, 2 ) * ny * nz + 2 * std::pow( ny, 3 ) * nz + 2 * ny * std::pow( nz, 3 ) ) /
            2.,
        1 - 2 * std::pow( nz, 2 ) + std::pow( nx, 2 ) * std::pow( nz, 2 ) + std::pow( ny, 2 ) * std::pow( nz, 2 ) +
            std::pow( nz, 4 );

    qi << -2 * ox + 4 * std::pow( nx, 2 ) * ox - 2 * std::pow( nx, 4 ) * ox -
              2 * std::pow( nx, 2 ) * std::pow( ny, 2 ) * ox - 2 * std::pow( nx, 2 ) * std::pow( nz, 2 ) * ox +
              4 * nx * ny * oy - 2 * std::pow( nx, 3 ) * ny * oy - 2 * nx * std::pow( ny, 3 ) * oy -
              2 * nx * ny * std::pow( nz, 2 ) * oy + 4 * nx * nz * oz - 2 * std::pow( nx, 3 ) * nz * oz -
              2 * nx * std::pow( ny, 2 ) * nz * oz - 2 * nx * std::pow( nz, 3 ) * oz,
        4 * nx * ny * ox - 2 * std::pow( nx, 3 ) * ny * ox - 2 * nx * std::pow( ny, 3 ) * ox -
            2 * nx * ny * std::pow( nz, 2 ) * ox - 2 * oy + 4 * std::pow( ny, 2 ) * oy -
            2 * std::pow( nx, 2 ) * std::pow( ny, 2 ) * oy - 2 * std::pow( ny, 4 ) * oy -
            2 * std::pow( ny, 2 ) * std::pow( nz, 2 ) * oy + 4 * ny * nz * oz - 2 * std::pow( nx, 2 ) * ny * nz * oz -
            2 * std::pow( ny, 3 ) * nz * oz - 2 * ny * std::pow( nz, 3 ) * oz,
        4 * nx * nz * ox - 2 * std::pow( nx, 3 ) * nz * ox - 2 * nx * std::pow( ny, 2 ) * nz * ox -
            2 * nx * std::pow( nz, 3 ) * ox + 4 * ny * nz * oy - 2 * std::pow( nx, 2 ) * ny * nz * oy -
            2 * std::pow( ny, 3 ) * nz * oy - 2 * ny * std::pow( nz, 3 ) * oy - 2 * oz + 4 * std::pow( nz, 2 ) * oz -
            2 * std::pow( nx, 2 ) * std::pow( nz, 2 ) * oz - 2 * std::pow( ny, 2 ) * std::pow( nz, 2 ) * oz -
            2 * std::pow( nz, 4 ) * oz;

    Q += Qi;
    q += qi;
  }
  Eigen::Matrix3d A = 2 * Q;
  Eigen::Vector3d x = A.colPivHouseholderQr().solve( -q );
  p[ 0 ]            = x[ 0 ];
  p[ 1 ]            = x[ 1 ];
  p[ 2 ]            = x[ 2 ];
  return x.hasNaN();
};

template <typename REAL>
bool intersect_plane3( const REAL *p0, const REAL *n0, const REAL *p1, const REAL *n1, REAL *o, REAL *t )
{
  constexpr REAL D_EPS = 1e-6;
  double d[ 3 ];

  d[ 0 ] = -n0[ 2 ] * n1[ 1 ] + n0[ 1 ] * n1[ 2 ];
  d[ 1 ] = n0[ 2 ] * n1[ 0 ] - n0[ 0 ] * n1[ 2 ];
  d[ 2 ] = -n0[ 1 ] * n1[ 0 ] + n0[ 0 ] * n1[ 1 ];

  double dnorm = std::sqrt( d[ 0 ] * d[ 0 ] + d[ 1 ] * d[ 1 ] + d[ 2 ] * d[ 2 ] );

  if ( dnorm < D_EPS ) return false;

  d[ 0 ] /= dnorm;
  d[ 1 ] /= dnorm;
  d[ 2 ] /= dnorm;

  double move_dir[ 3 ];

  move_dir[ 0 ] = -d[ 2 ] * n0[ 1 ] + d[ 1 ] * n0[ 2 ];
  move_dir[ 1 ] = d[ 2 ] * n0[ 0 ] - d[ 0 ] * n0[ 2 ];
  move_dir[ 2 ] = -d[ 1 ] * n0[ 0 ] + d[ 0 ] * n0[ 1 ];

  double lambda = ( p1[ 0 ] - p0[ 0 ] ) * n1[ 0 ] + ( p1[ 1 ] - p0[ 1 ] ) * n1[ 1 ] + ( p1[ 2 ] - p0[ 2 ] ) * n1[ 2 ];
  lambda /= ( move_dir[ 0 ] * n1[ 0 ] + move_dir[ 1 ] * n1[ 1 ] + move_dir[ 2 ] * n1[ 2 ] );

  t[ 0 ] = d[ 0 ];
  t[ 1 ] = d[ 1 ];
  t[ 2 ] = d[ 2 ];

  o[ 0 ] = p0[ 0 ] + move_dir[ 0 ] * lambda;
  o[ 1 ] = p0[ 1 ] + move_dir[ 1 ] * lambda;
  o[ 2 ] = p0[ 2 ] + move_dir[ 2 ] * lambda;

  return true;
};

template <typename REAL>
REAL distance_ray3_tri3( const REAL *origin, const REAL *direction, const REAL *p0, const REAL *p1, const REAL *p2,
                         const REAL &nearest )
{
  REAL e1[ 3 ], e2[ 3 ], s1[ 3 ];

  for ( size_t i = 0; i < 3; i++ )
  {
    e1[ i ] = p1[ i ] - p0[ i ];
    e2[ i ] = p2[ i ] - p0[ i ];
  }

  const REAL zero = std::numeric_limits<REAL>::epsilon() * (REAL)2.0;
  const REAL one  = (REAL)1;

  s1[ 0 ] = ( direction[ 1 ] * e2[ 2 ] );
  s1[ 0 ] -= direction[ 2 ] * e2[ 1 ];
  s1[ 1 ] = ( direction[ 2 ] * e2[ 0 ] );
  s1[ 1 ] -= direction[ 0 ] * e2[ 2 ];
  s1[ 2 ] = ( direction[ 0 ] * e2[ 1 ] );
  s1[ 2 ] -= direction[ 1 ] * e2[ 0 ];

  REAL divisor = s1[ 0 ] * e1[ 0 ] + s1[ 1 ] * e1[ 1 ] + s1[ 2 ] * e1[ 2 ];

  // printf( "99999\n" );

  if ( std::abs( divisor ) <= zero ) return nearest;

  REAL inv_divisor = one / divisor;

  REAL d[ 3 ];
  for ( int i = 0; i < 3; i++ ) d[ i ] = origin[ i ] - p0[ i ];

  REAL b0 = ( d[ 0 ] * s1[ 0 ] + d[ 1 ] * s1[ 1 ] + d[ 2 ] * s1[ 2 ] ) * inv_divisor;
  // printf( "11111\n" );
  if ( b0 <= zero || b0 > one ) return nearest;
  REAL s2[ 3 ];

  s2[ 0 ] = ( d[ 1 ] * e1[ 2 ] );
  s2[ 0 ] -= d[ 2 ] * e1[ 1 ];
  s2[ 1 ] = ( d[ 2 ] * e1[ 0 ] );
  s2[ 1 ] -= d[ 0 ] * e1[ 2 ];
  s2[ 2 ] = ( d[ 0 ] * e1[ 1 ] );
  s2[ 2 ] -= d[ 1 ] * e1[ 0 ];

  REAL b1 = ( direction[ 0 ] * s2[ 0 ] + direction[ 1 ] * s2[ 1 ] + direction[ 2 ] * s2[ 2 ] ) * inv_divisor;
  // printf( "2222\n" );
  if ( b1 <= zero || b0 + b1 > one ) return nearest;

  REAL t = inv_divisor * ( e2[ 0 ] * s2[ 0 ] + e2[ 1 ] * s2[ 1 ] + e2[ 2 ] * s2[ 2 ] );
  // printf( "3333\n" );
  if ( t < zero ) return nearest;
  // printf( "ONLY %f\n", t );

  return std::min( nearest, t );
  // constexpr REAL zero = std::numeric_limits<REAL>::epsilon() * (REAL)2.0;
  // constexpr REAL one  = (REAL)1;
  // REAL e1[ 3 ], e2[ 3 ], s1[ 3 ];
  // e1[ 0 ] = p1[ 0 ] - p0[ 0 ];
  // e2[ 0 ] = p2[ 0 ] - p0[ 0 ];
  // e1[ 1 ] = p1[ 1 ] - p0[ 1 ];
  // e2[ 1 ] = p2[ 1 ] - p0[ 1 ];
  // e1[ 2 ] = p1[ 2 ] - p0[ 2 ];
  // e2[ 2 ] = p2[ 2 ] - p0[ 2 ];

  // s1[ 0 ] = ( direction[ 1 ] * e2[ 2 ] );
  // s1[ 0 ] -= direction[ 2 ] * e2[ 1 ];
  // s1[ 1 ] = ( direction[ 2 ] * e2[ 0 ] );
  // s1[ 1 ] -= direction[ 0 ] * e2[ 2 ];
  // s1[ 2 ] = ( direction[ 0 ] * e2[ 1 ] );
  // s1[ 2 ] -= direction[ 1 ] * e2[ 0 ];

  // REAL divisor = s1[ 0 ] * e1[ 0 ] + s1[ 1 ] * e1[ 1 ] + s1[ 2 ] * e1[ 2 ];

  // if ( std::abs( divisor ) <= zero ) return nearest;

  // REAL inv_divisor = one / divisor;

  // REAL d[ 3 ];
  // for ( int i = 0; i < 3; i++ ) d[ i ] = origin[ i ] - p0[ i ];

  // REAL b0 = ( d[ 0 ] * s1[ 0 ] + d[ 1 ] * s1[ 1 ] + d[ 2 ] * s1[ 2 ] ) * inv_divisor;

  // if ( b0 <= zero || b0 > one ) return nearest;
  // REAL s2[ 3 ];

  // s2[ 0 ] = ( d[ 1 ] * e1[ 2 ] );
  // s2[ 0 ] -= d[ 2 ] * e1[ 1 ];
  // s2[ 1 ] = ( d[ 2 ] * e1[ 0 ] );
  // s2[ 1 ] -= d[ 0 ] * e1[ 2 ];
  // s2[ 2 ] = ( d[ 0 ] * e1[ 1 ] );
  // s2[ 2 ] -= d[ 1 ] * e1[ 0 ];

  // REAL b1 = ( direction[ 0 ] * s2[ 0 ] + direction[ 1 ] * s2[ 1 ] + direction[ 2 ] * s2[ 2 ] ) * inv_divisor;
  // if ( b1 <= zero || b0 + b1 > one ) return nearest;

  // REAL t = inv_divisor * ( e2[ 0 ] * s2[ 0 ] + e2[ 1 ] * s2[ 1 ] + e2[ 2 ] * s2[ 2 ] );
  // if ( t < zero ) return nearest;

  // return std::min( nearest, t );
};

template <typename REAL>
void bounding_aabb_tri3( const REAL *v0, const REAL *v1, const REAL *v2, REAL *vmin, REAL *vmax, REAL epsilon )
{
  vmin[ 0 ] = std::min( v0[ 0 ], std::min( v1[ 0 ], v2[ 0 ] ) );
  vmin[ 1 ] = std::min( v0[ 1 ], std::min( v1[ 1 ], v2[ 1 ] ) );
  vmin[ 2 ] = std::min( v0[ 2 ], std::min( v1[ 2 ], v2[ 2 ] ) );
  vmax[ 0 ] = std::max( v0[ 0 ], std::max( v1[ 0 ], v2[ 0 ] ) );
  vmax[ 1 ] = std::max( v0[ 1 ], std::max( v1[ 1 ], v2[ 1 ] ) );
  vmax[ 2 ] = std::max( v0[ 2 ], std::max( v1[ 2 ], v2[ 2 ] ) );
};

template <typename FT>
FT DistanceQuery_PointLineSegment( const FT *p, const FT *v0, const FT *v1, FT *v_closest, int dim )
{
  const int MAX_DIM = 3;
  FT lambda0[ MAX_DIM ];

  return DistanceQuery_PointLineSegment( p, v0, v1, v_closest, dim, lambda0 );
};
template <typename FT>
FT DistanceQuery_PointLineSegment( const FT *point, const FT *V0, const FT *V1, FT *closest_point, int dim,
                                   FT *lambda0 )
{
  using Scalar = FT;

  Scalar l2 = 0, t = 0;
  int i     = 0;
  Scalar ds = 0;
  for ( i = 0; i < dim; i++ )
  {
    l2 += ( V0[ i ] - V1[ i ] ) * ( V0[ i ] - V1[ i ] );
    t += ( point[ i ] - V0[ i ] ) * ( V1[ i ] - V0[ i ] );
  }

  if ( t <= Scalar( 0 ) || l2 == Scalar( 0 ) )
  {
    for ( i = 0; i < dim; i++ ) closest_point[ i ] = V0[ i ];
    *lambda0 = Scalar( 1 );
    ds       = 0;
    for ( i = 0; i < dim; i++ ) ds += ( point[ i ] - V0[ i ] ) * ( point[ i ] - V0[ i ] );
    return ds;
  }
  else if ( t > l2 )
  {
    for ( i = 0; i < dim; i++ ) closest_point[ i ] = V1[ i ];
    *lambda0 = Scalar( 0 );
    // lambda1 = Scalar( 1 );
    ds = 0;
    for ( i = 0; i < dim; i++ ) ds += ( point[ i ] - V1[ i ] ) * ( point[ i ] - V1[ i ] );
    return ds;
  }
  Scalar lambda1 = t / l2;
  *lambda0       = Scalar( 1 ) - lambda1;
  for ( i = 0; i < dim; i++ ) closest_point[ i ] = ( *lambda0 ) * V0[ i ] + lambda1 * V1[ i ];
  ds = 0;
  for ( i = 0; i < dim; i++ ) ds += ( point[ i ] - closest_point[ i ] ) * ( point[ i ] - closest_point[ i ] );
  return ds;
};

template <typename FT>
FT distance_point3_segment3( const FT *u0, const FT *u1, const FT *v0, const FT *v1, FT *u_closest, FT *v_closest,
                             int dim )
{
  const int MAX_DIM = 3;
  FT lambdaU[ MAX_DIM ];
  FT lambdaV[ MAX_DIM ];

  return distance_point3_segment3( u0, u1, v0, v1, u_closest, v_closest, dim, lambdaU, lambdaV );
};
template <typename FT>
FT distance_point3_segment3( const FT *U0, const FT *U1, const FT *V0, const FT *V1, FT *closest_pointU,
                             FT *closest_pointV, int dim, FT *lambdaU, FT *lambdaV )
{
  using Scalar          = FT;
  constexpr Scalar ZERO = static_cast<Scalar>( 0 );
  constexpr Scalar ONE  = static_cast<Scalar>( 1 );
  constexpr Scalar EPS  = std::numeric_limits<Scalar>::epsilon();

  constexpr bool is_lineU = false;
  constexpr bool is_lineV = false;

  const int MAX_DIM = 3;

  Scalar d1[ MAX_DIM ];
  Scalar d2[ MAX_DIM ];
  Scalar r[ MAX_DIM ];
  Scalar ds = 0;
  int i;

  for ( i = 0; i < dim; i++ )
  {
    d1[ i ] = U1[ i ] - U0[ i ];
    d2[ i ] = V1[ i ] - V0[ i ];
    r[ i ]  = U0[ i ] - V0[ i ];
  }

  Scalar a = 0, e = 0, f = 0;
  for ( i = 0; i < dim; i++ )
  {
    a += d1[ i ] * d1[ i ];
    e += d2[ i ] * d2[ i ];
    f += d2[ i ] * r[ i ];
  }

  if ( std::abs( a ) < EPS )
  {
    if ( std::abs( e ) < EPS )
    {
      ( *lambdaU ) = ( *lambdaV ) = 0;

      for ( i = 0; i < dim; i++ )
      {
        closest_pointU[ i ] = U0[ i ];
        closest_pointV[ i ] = V0[ i ];
      }

      ds = 0;
      for ( i = 0; i < dim; i++ )
        ds += ( closest_pointU[ i ] - closest_pointV[ i ] ) * ( closest_pointU[ i ] - closest_pointV[ i ] );

      return ds;
    }
    else
    {
      *lambdaU = 0;
      *lambdaV = f / e;

      if ( !is_lineV ) *lambdaV = std::clamp( *lambdaV, ZERO, ONE );
    }
  }

  else
  {
    Scalar c = 0;
    for ( i = 0; i < dim; i++ ) c += d1[ i ] * r[ i ];

    if ( std::abs( e ) < EPS )
    {
      *lambdaV = 0;
      *lambdaU = -c / a;

      if ( !is_lineU ) *lambdaU = std::clamp( *lambdaU, ZERO, ONE );
    }
    else
    {
      Scalar b = 0;

      for ( i = 0; i < dim; i++ ) b += d1[ i ] * d2[ i ];
      Scalar denom = a * e - b * b;

      if ( std::abs( denom ) >= EPS )
      {
        *lambdaU = ( b * f - c * e ) / denom;

        if ( !is_lineU ) *lambdaU = std::clamp( *lambdaU, ZERO, ONE );
      }
      else
        *lambdaU = 0;

      *lambdaV = ( b * ( *lambdaU ) + f ) / e;

      if ( !is_lineV )
      {
        if ( *lambdaV < 0 )
        {
          *lambdaV = 0;
          *lambdaU = -c / a;

          if ( !is_lineU ) *lambdaU = std::clamp( *lambdaU, ZERO, ONE );
        }
        else if ( *lambdaV > 1 )
        {
          *lambdaV = 1;
          *lambdaU = ( b - c ) / a;

          if ( !is_lineU ) *lambdaU = std::clamp( *lambdaU, ZERO, ONE );
        }
      }
    }
  }
  for ( i = 0; i < dim; i++ )
  {
    closest_pointU[ i ] = 0;
    closest_pointV[ i ] = 0;
  }

  for ( i = 0; i < dim; i++ )
  {
    closest_pointU[ i ] += U0[ i ] + ( *lambdaU ) * d1[ i ];
    closest_pointV[ i ] += V0[ i ] + ( *lambdaV ) * d2[ i ];
  }

  ds = 0;
  for ( i = 0; i < dim; i++ )
    ds += ( closest_pointU[ i ] - closest_pointV[ i ] ) * ( closest_pointU[ i ] - closest_pointV[ i ] );

  return ds;
};

}  // namespace ampl