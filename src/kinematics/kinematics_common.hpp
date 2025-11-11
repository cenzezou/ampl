#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <cmath>
#define QT_SIZE 7
#define print_bits( x )                                                 \
  do                                                                    \
  {                                                                     \
    unsigned long long a__ = ( x );                                     \
    size_t bits__          = sizeof( x ) * 8;                           \
    printf( #x ": " );                                                  \
    while ( bits__-- ) putchar( a__ & ( 1ULL << bits__ ) ? '1' : '0' ); \
    putchar( '\n' );                                                    \
  } while ( 0 )

#define TWO_PI 6.28318530717958647693
namespace ampl::fast_math
{
std::array<std::array<double, 6>, 5> constexpr C_SIN_EST_COEFF = {
    { { // degree 3
        +1.0, -1.4727245910375519e-1 },
      { // degree 5
        +1.0, -1.6600599923812209e-1, +7.5924178409012000e-3 },
      { // degree 7
        +1.0, -1.6665578084732124e-1, +8.3109378830028557e-3, -1.8447486103462252e-4 },
      { // degree 9
        +1.0, -1.6666656235308897e-1, +8.3329962509886002e-3, -1.9805100675274190e-4, +2.5967200279475300e-6 },
      { // degree 11
        +1.0, -1.6666666601721269e-1, +8.3333303183525942e-3, -1.9840782426250314e-4, +2.7521557770526783e-6,
        -2.3828544692960918e-8 } } };

std::array<double, 5> constexpr C_SIN_EST_MAX_ERROR = {
    1.3481903639146e-2,  // degree 3
    1.4001209384651e-4,  // degree 5
    1.0205878939740e-6,  // degree 7
    5.2010783457846e-9,  // degree 9
    1.9323431743601e-11  // degree 11
};
std::array<std::array<double, 6>, 5> constexpr C_COS_EST_COEFF = {
    { { // degree 2
        +1.0, -4.0528473456935105e-1 },
      { // degree 4
        +1.0, -4.9607181958647262e-1, +3.6794619653489236e-2 },
      { // degree 6
        +1.0, -4.9992746217057404e-1, +4.1493920348353308e-2, -1.2712435011987822e-3 },
      { // degree 8
        +1.0, -4.9999925121358291e-1, +4.1663780117805693e-2, -1.3854239405310942e-3, +2.3154171575501259e-5 },
      { // degree 10
        +1.0, -4.9999999508695869e-1, +4.1666638865338612e-2, -1.3888377661039897e-3, +2.4760495088926859e-5,
        -2.6051615464872668e-7 } } };

template <typename T, size_t Degree>
inline T SinOverXEstimate( T x )
{
  static_assert( ( Degree & 1 ) == 1 && 1 <= ( ( Degree - 1 ) / 2 ) && ( ( Degree - 1 ) / 2 ) <= 5, "Invalid degree." );

  size_t constexpr select = ( ( Degree - 3 ) / 2 );
  auto constexpr &coeff   = C_SIN_EST_COEFF[ select ];
  size_t constexpr last   = ( ( Degree - 1 ) / 2 );
  T xsqr                  = x * x;
  T poly                  = static_cast<T>( coeff[ last ] );
  for ( size_t i = 0, index = last - 1; i < last; ++i, --index )
  {
    poly = static_cast<T>( coeff[ index ] ) + poly * xsqr;
  }
  return poly;
}
template <typename T, size_t Degree>
inline T OneMinusCosOverXXEstimate( T x )
{
  static_assert( ( Degree & 1 ) == 0 && 1 <= ( Degree / 2 ) && ( Degree / 2 ) <= 5, "Invalid degree." );

  size_t constexpr select = ( Degree - 2 ) / 2;
  auto constexpr &coeff   = C_COS_EST_COEFF[ select ];
  size_t constexpr last   = ( Degree / 2 );
  T xsqr                  = x * x;
  T poly                  = static_cast<T>( coeff[ last ] );
  for ( size_t i = 0, index = last - 1; i < last - 1; ++i, --index )
  {
    poly = static_cast<T>( coeff[ index ] ) + poly * xsqr;
  }
  return -poly;
}
template <typename T, size_t Degree>
inline T OneMinusCosOverXXEstimateRR( T x )
{
  static_assert( ( Degree & 1 ) == 0 && 1 <= ( Degree / 2 ) && ( Degree / 2 ) <= 5, "Invalid degree." );

  // Map x to r in [-pi,pi].
  T r = std::remainder( x, static_cast<T>( M_PI * 2 ) );

  // Map r to y in [-pi/2,pi/2] with cos(y) = sign * cos(x).
  T const halfPi = static_cast<T>( M_PI_2 );
  if ( r > halfPi )
  {
    // r is in (pi/2,pi], so y = pi - r is in (-pi/2,0], sign = -1
    return -OneMinusCosOverXXEstimate<T, Degree>( static_cast<T>( M_PI ) - r );
  }
  else if ( r < -halfPi )
  {
    // r is in [-pi,-pi/2), so y = -pi - r is in [0,pi/2), sign = -1
    return -OneMinusCosOverXXEstimate<T, Degree>( static_cast<T>( -M_PI ) - r );
  }
  else
  {
    // r is in [-pi/2,pi/2], y = r, sign = +1
    return OneMinusCosOverXXEstimate<T, Degree>( r );
  }
}

template <typename T, size_t Degree>
inline T SinOverXEstimateRR( T x )
{
  static_assert( ( Degree & 1 ) == 1 && 1 <= ( ( Degree - 1 ) / 2 ) && ( ( Degree - 1 ) / 2 ) <= 5, "Invalid degree." );

  // Map x to r in [-pi,pi].
  T r = std::remainder( x, static_cast<T>( M_PI * 2 ) );
  // Map r to y in [-pi/2,pi/2] with sin(y) = sin(x).
  T const halfPi = static_cast<T>( M_PI_2 );
  if ( r > halfPi )
  {
    // r is in (pi/2,pi], so y = pi - r is in (-pi/2,0]
    return SinOverXEstimate<T, Degree>( static_cast<T>( M_PI ) - r );
  }
  else if ( r < -halfPi )
  {
    // r is in [-pi,-pi/2), so y = -pi - r is in [0,pi/2)
    return SinOverXEstimate<T, Degree>( static_cast<T>( -M_PI ) - r );
  }
  else
  {
    // r is in [-pi/2,pi/2], y = r
    return SinOverXEstimate<T, Degree>( r );
  }
}

}  // namespace ampl::fast_math

namespace ampl
{
template <typename REAL>
REAL distance_point_line3( const REAL *o, const REAL *n, const REAL *p )
{
  double op[ 3 ] = { p[ 0 ] - o[ 0 ], p[ 1 ] - o[ 1 ], p[ 2 ] - o[ 2 ] };
  return std::pow( n[ 1 ] * op[ 0 ] - n[ 0 ] * op[ 1 ], 2 ) + std::pow( -( n[ 2 ] * op[ 0 ] ) + n[ 0 ] * op[ 2 ], 2 ) +
         std::pow( n[ 2 ] * op[ 1 ] - n[ 1 ] * op[ 2 ], 2 );
};
template <typename REAL>
bool intersect_line3( const REAL *os, const REAL *ns, uint32_t num_lines, REAL *p )
{
  Eigen::Matrix3d Q;
  Q.setZero();
  Eigen::Vector3d q;
  q.setZero();

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
}  // namespace ampl

namespace ampl
{
template <typename REAL>
static void xyzrpy2tf( const REAL *xyz, const REAL *rpy, Eigen::Matrix4d &Tf )
{
  Tf.setIdentity();
  Tf( 0, 3 )               = xyz[ 0 ];
  Tf( 1, 3 )               = xyz[ 1 ];
  Tf( 2, 3 )               = xyz[ 2 ];
  Tf.topLeftCorner<3, 3>() = Eigen::AngleAxisd( rpy[ 2 ], Eigen::Vector3d::UnitZ() ).matrix() *
                             Eigen::AngleAxisd( rpy[ 1 ], Eigen::Vector3d::UnitY() ).matrix() *
                             Eigen::AngleAxisd( rpy[ 0 ], Eigen::Vector3d::UnitX() ).matrix();
}

template <uint32_t K, typename REAL>
static const std::array<Eigen::Matrix4d, K> urdf2tfs( const REAL *urdf )
{
  Eigen::Matrix4d tf_cache = Eigen::Matrix4d::Identity();
  std::array<Eigen::Matrix4d, K> tfs;
  for ( auto k = 0U; k < K; k++ )
  {
    const REAL *xyzrpy = &urdf[ 9 * k ];
    xyzrpy2tf<REAL>( xyzrpy, xyzrpy + 3, tf_cache );
    tfs[ k ] = tf_cache;
  }
  return tfs;
};

template <uint32_t K, typename REAL>
static const std::array<Eigen::Vector<double, 6>, K> urdf2twists( const REAL *urdf )
{
  std::array<Eigen::Vector<double, 6>, K> out;
  for ( auto k = 0U; k < K; k++ )
  {
    out[ k ].template head<3>().setZero();
    out[ k ].template tail<3>() = Eigen::Vector3d( urdf[ 9 * k + 6 ], urdf[ 9 * k + 7 ], urdf[ 9 * k + 8 ] );
  }
  return out;
};

template <uint32_t K>
static const Eigen::Matrix4d tfs2eehome( const Eigen::Matrix4d *tfs_cache )
{
  Eigen::Matrix4d tf_ee = Eigen::Matrix4d::Identity();
  for ( auto k = 0U; k < K; k++ ) tf_ee *= tfs_cache[ k ];
  return tf_ee;
};

template <uint32_t K, typename REAL>
static const std::array<Eigen::Vector<double, 6>, K> tfs2pn( const Eigen::Matrix4d *tfs_cache, const REAL *urdf )
{
  Eigen::Matrix4d tf_ee = Eigen::Matrix4d::Identity();
  std::array<Eigen::Vector<double, 6>, K> out;

  for ( auto k = 0U; k < K; k++ )
  {
    tf_ee *= tfs_cache[ k ];

    out[ k ].template head<3>() = tf_ee.topRightCorner<3, 1>();
    out[ k ].template tail<3>() =
        tf_ee.topLeftCorner<3, 3>() * Eigen::Vector3d( urdf[ 9 * k + 6 ], urdf[ 9 * k + 7 ], urdf[ 9 * k + 8 ] );
  }

  return out;
};

template <typename REAL>
inline void SE3_inv( const Eigen::Matrix4<REAL> &SE3, Eigen::Matrix4<REAL> &T )
{
  T.template topLeftCorner<3, 3>() = SE3.template topLeftCorner<3, 3>().transpose();
  T.template topRightCorner<3, 1>() =
      -SE3.template topLeftCorner<3, 3>().transpose() * SE3.template topRightCorner<3, 1>();
  T.row( 3 ).setZero();
  T( 3, 3 ) = 1.0;
};
template <typename REAL>
inline void so3_up( const Eigen::Vector3<REAL> &omg, Eigen::Ref<Eigen::Matrix3<REAL>> so3 )
{
  so3 << 0, -omg( 2 ), omg( 1 ), omg( 2 ), 0, -omg( 0 ), -omg( 1 ), omg( 0 ), 0;
};

template <typename REAL>
inline void so3_up2( const Eigen::Vector3<REAL> &omg, Eigen::Matrix3<REAL> &so3 )
{
  so3 << -( omg( 1 ) * omg( 1 ) ) - ( omg( 2 ) * omg( 2 ) ), omg( 0 ) * omg( 1 ), omg( 0 ) * omg( 2 ),
      omg( 0 ) * omg( 1 ), -( omg( 0 ) * omg( 0 ) ) - ( omg( 2 ) * omg( 2 ) ), omg( 1 ) * omg( 2 ), omg( 0 ) * omg( 2 ),
      omg( 1 ) * omg( 2 ), -( omg( 0 ) * omg( 0 ) ) - ( omg( 1 ) * omg( 1 ) );
}
template <typename REAL>
inline void SE3_Adj( const Eigen::Matrix4<REAL> &T, Eigen::Matrix<REAL, 6, 6> &A )
{
  Eigen::Matrix3<REAL> so3 = Eigen::Matrix3<REAL>::Zero();
  so3_up<REAL>( T.template topRightCorner<3, 1>(), so3 );
  A.setZero();
  A.template topLeftCorner<3, 3>()     = T.template topLeftCorner<3, 3>();
  A.template bottomRightCorner<3, 3>() = T.template topLeftCorner<3, 3>();
  A.template topRightCorner<3, 3>()    = so3 * T.template topLeftCorner( 3, 3 );
  A.template bottomLeftCorner<3, 3>().setZero();
};
template <typename REAL>
inline bool near_zero( const REAL &val )
{
  return ( std::abs( val ) < (REAL)1e-6 );
}
inline void so3_upexp( const Eigen::Vector3d &u, Eigen::Ref<Eigen::Matrix3d> SO3 )
{
  const double theta = u.norm();
  if ( near_zero( theta ) )
  {
    SO3.setIdentity();
    return;
  }
  else
  {
    // double oneminusc = (1.0 - std::cos(theta)) / theta / theta;
    const bool is_theta_small = theta < 0.1;
    double s = is_theta_small ? fast_math::SinOverXEstimateRR<double, 11>( theta ) : std::sin( theta ) / theta;

    double oneminusc = is_theta_small ? fast_math::OneMinusCosOverXXEstimate<double, 10>( theta )
                                      : ( 1.0 - std::cos( theta ) ) / theta / theta;
    // double s = std::sin(theta) / theta;

    SO3 << 1 + oneminusc * ( -std::pow( u( 1 ), 2 ) - std::pow( u( 2 ), 2 ) ), oneminusc * u( 0 ) * u( 1 ) - s * u( 2 ),
        s * u( 1 ) + oneminusc * u( 0 ) * u( 2 ), oneminusc * u( 0 ) * u( 1 ) + s * u( 2 ),
        1.0 + oneminusc * ( -std::pow( u( 0 ), 2 ) - std::pow( u( 2 ), 2 ) ),
        -( s * u( 0 ) ) + oneminusc * u( 1 ) * u( 2 ), -( s * u( 1 ) ) + oneminusc * u( 0 ) * u( 2 ),
        s * u( 0 ) + oneminusc * u( 1 ) * u( 2 ), 1.0 + oneminusc * ( -std::pow( u( 0 ), 2 ) - std::pow( u( 1 ), 2 ) );
  };
}

inline void se3_upexp( const Eigen::Vector<double, 6> &rw, Eigen::Ref<Eigen::Matrix4d> SE3 )
{
  double theta = rw.tail<3>().norm();

  if ( near_zero( theta ) )
  {
    SE3.topRightCorner<3, 1>() = rw.head<3>();
    return;
  }
  else
  {
    const bool is_theta_small = theta < 0.1;
    double oneminusc          = is_theta_small ? fast_math::OneMinusCosOverXXEstimate<double, 10>( theta )
                                               : ( 1.0 - std::cos( theta ) ) / theta / theta;
    double thetaminuss        = ( theta - std::sin( theta ) ) / theta / theta / theta;

    so3_upexp( rw.tail<3>(), SE3.topLeftCorner<3, 3>() );
    Eigen::Vector3d w_cross_r = rw.tail<3>().cross( rw.head<3>() );
    SE3.topRightCorner<3, 1>() =
        rw.head<3>() + oneminusc * w_cross_r + thetaminuss * ( rw.tail<3>().cross( w_cross_r ) );
    SE3.row( 3 ).setZero();
    SE3( 3, 3 ) = 1.0;
  }
};

template <uint32_t K>
static const std::array<Eigen::Vector<double, 6>, K> find_poetwists( const Eigen::Matrix4d *tfs_cache,
                                                                     const Eigen::Vector<double, 6> *twists )
{
  std::array<Eigen::Vector<double, 6>, K> out;
  Eigen::Matrix4d tf_ee = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 6, 6> A;
  for ( auto k = 0U; k < K; k++ )
  {
    tf_ee *= tfs_cache[ k ];
    SE3_Adj<double>( tf_ee, A );
    out[ k ] = A * twists[ k ];
  }
  return out;
};

static const Eigen::Vector3d find_wrist( const Eigen::Vector<double, 6> *pns, const uint32_t *ids )
{
  Eigen::Vector3d p_wrist;
  double os[ 9 ] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  double ns[ 9 ] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  for ( int i = 0; i < 3; i++ )
  {
    if ( ids[ i ] > 7 ) continue;

    const double *oi = pns[ ids[ i ] ].head<3>().data();
    const double *ni = pns[ ids[ i ] ].tail<3>().data();
    for ( int k = 0; k < 3; k++ )
    {
      os[ i * 3 + k ] = oi[ k ];
      ns[ i * 3 + k ] = ni[ k ];
    }
  }

  if ( ids[ 2 ] > 7 )
  {
    intersect_line3<double>( os, ns, 2, p_wrist.data() );
  }
  else
    intersect_line3<double>( os, ns, 3, p_wrist.data() );
  Eigen::Vector3d dist_to_joint;
  dist_to_joint.setOnes();

  dist_to_joint[ 0 ] = distance_point_line3<double>( pns[ ids[ 0 ] ].head<3>().data(), pns[ ids[ 0 ] ].tail<3>().data(),
                                                     p_wrist.data() );

  dist_to_joint[ 1 ] = distance_point_line3<double>( pns[ ids[ 1 ] ].head<3>().data(), pns[ ids[ 1 ] ].tail<3>().data(),
                                                     p_wrist.data() );

  if ( ids[ 2 ] <= 7 )
  {
    dist_to_joint[ 2 ] = distance_point_line3<double>( pns[ ids[ 2 ] ].head<3>().data(),
                                                       pns[ ids[ 2 ] ].tail<3>().data(), p_wrist.data() );
  }

  // std::cout << p_wrist << std::endl;
  // std::cout << dist_to_joint << std::endl;
  return p_wrist;
}

template <uint32_t K, typename REAL, typename REAL_QT>
static const std::array<REAL_QT, K * 7> urdf2qts( const REAL *urdf )
{
  Eigen::Matrix4d tf_cache = Eigen::Matrix4d::Identity();
  std::array<REAL_QT, K * 7> qts;
  for ( auto k = 0U; k < K; k++ )
  {
    const REAL *xyzrpy = &urdf[ 9 * k ];
    xyzrpy2tf<REAL>( xyzrpy, xyzrpy + 3, tf_cache );
    Eigen::Quaterniond quat( tf_cache.topLeftCorner<3, 3>() );
    Eigen::Vector3d trans = tf_cache.topRightCorner<3, 1>();
    // rwt
    qts[ k * 7 + 0 ] = quat.coeffs()( 0 );
    qts[ k * 7 + 1 ] = quat.coeffs()( 1 );
    qts[ k * 7 + 2 ] = quat.coeffs()( 2 );
    qts[ k * 7 + 3 ] = quat.coeffs()( 3 );
    qts[ k * 7 + 4 ] = trans( 0 );
    qts[ k * 7 + 5 ] = trans( 1 );
    qts[ k * 7 + 6 ] = trans( 2 );
  }
  return qts;
};

template <uint32_t K, typename REAL>
static const std::array<double, K * 3> urdf2axisarr( const REAL *urdf )
{
  std::array<double, K * 3> out;
  for ( auto k = 0U; k < K; k++ )
  {
    out[ 3 * k + 0 ] = urdf[ 9 * k + 6 ];
    out[ 3 * k + 1 ] = urdf[ 9 * k + 7 ];
    out[ 3 * k + 2 ] = urdf[ 9 * k + 8 ];
  }
  return out;
};

bool subproblem_RRR( const Eigen::Matrix3d &R, const Eigen::Vector3d &w1, const Eigen::Vector3d &w2,
                     const Eigen::Vector3d &w3, double *thA, double *thB );

bool subproblem_PK1( const Eigen::Vector3d &p, const Eigen::Vector3d &q, const Eigen::Vector3d &w,
                     const Eigen::Vector3d &r, double *th );

bool subproblem_PK3( const Eigen::Vector3d &p, const Eigen::Vector3d &q, const Eigen::Vector3d &r,
                     const Eigen::Vector3d &w, const double *delta, double *th_p, double *th_m );

void subproblem_PK2( const Eigen::Vector3d &p, const Eigen::Vector3d &q, const Eigen::Vector3d &r,
                     const Eigen::Vector3d &w1, const Eigen::Vector3d &w2, double *th1, double *th2, bool *status );

}  // namespace ampl