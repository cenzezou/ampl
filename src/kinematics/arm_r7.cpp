

#include "arms.hpp"
#include "arm_database.hpp"
#include "kinematics_common.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ampl/kinematics.hpp>
#include <iostream>
#include <memory>

namespace ampl
{

void ArmR7::initialize_urdf( const double *urdf, const double *joint_limits, const double *xyzrpy_tool0 )
{
  tfs_cache        = urdf2tfs<K, double>( urdf );
  twists_home      = urdf2twists<K, double>( urdf );
  link_end_fk_home = tfs2eehome<K>( tfs_cache.data() );
  pns_joint        = tfs2pn<K, double>( tfs_cache.data(), urdf );
  twists_poe       = find_poetwists<K>( tfs_cache.data(), twists_home.data() );

  constexpr uint32_t ids_wrist[ 3 ]    = { 4, 5, 1000 };
  constexpr uint32_t ids_shoulder[ 3 ] = { 0, 1, 2 };
  q_wrist                              = find_wrist( pns_joint.data(), ids_wrist );
  q_shoulder                           = find_wrist( pns_joint.data(), ids_shoulder );

  qts_cache_f  = urdf2qts<K, double, float>( urdf );
  qts_cache    = urdf2qts<K, double, double>( urdf );
  twists_cache = urdf2axisarr<K, double>( urdf );

  if ( joint_limits )
  {
    memcpy( q_lo.data(), joint_limits, K * sizeof( double ) );
    memcpy( q_hi.data(), joint_limits + K, K * sizeof( double ) );
  }
  else
  {
    q_lo.setConstant( -M_PI );
    q_hi.setConstant( M_PI );
  }

  tf_link_end_tool0.setIdentity();
  tf_tool0_link_end.setIdentity();
  tf_link_end_tcp.setIdentity();

  if ( xyzrpy_tool0 )
  {
    set_link_end_tool0( xyzrpy_tool0 );
  }
  tf_link_end_tcp = tf_link_end_tool0;
}
void ArmR7::initialize_preset( const std::string &name )
{
  if ( name == "hillbot_left" ) initialize_urdf( HILLBOT_LEFT::urdf, HILLBOT_LEFT::bounds, HILLBOT_LEFT::tool0 );
};
void ArmR7::set_joint_limits( const double *joint_limits_low, const double *joint_limits_hi )
{
  memcpy( q_lo.data(), joint_limits_low, K * sizeof( double ) );
  memcpy( q_hi.data(), joint_limits_hi, K * sizeof( double ) );
};

void ArmR7::set_link_end_tool0( const double *xyzrpy )
{
  xyzrpy2tf<double>( xyzrpy, xyzrpy + 3, tf_link_end_tool0 );

  quat_end_tool0 = quatd( tf_link_end_tool0.topLeftCorner<3, 3>() );
  t_end_tool0    = tf_link_end_tool0.topRightCorner<3, 1>();

  SE3_inv<double>( tf_link_end_tool0, tf_tool0_link_end );
};
void ArmR7::set_tcp( const double *tf44, bool colmajor )
{
  if ( colmajor )
  {
    Eigen::Map<const Mat4d> tf_tool0_tcp( tf44 );
    tf_link_end_tcp = tf_link_end_tool0 * tf_tool0_tcp;
  }
  else
  {
    tf_link_end_tcp = Eigen::Map<const Mat4d>( tf44 );
    tf_link_end_tcp.transposeInPlace();
    tf_link_end_tcp = tf_link_end_tool0 * tf_link_end_tcp;
  }
};
void ArmR7::get_pose_tool0( double *tf44, bool colmajor ){};

void ArmR7::fk( const double *q, double *qts_link )
{
  double *qt = qts_link;
  double *qtm1;
  Eigen::Map<const Eigen::Quaterniond> quat0_cache( qts_cache.data() );
  Eigen::Map<const Vec3d> axis0_cache( twists_cache.data() );
  Eigen::Map<Eigen::Quaterniond> quat0( qt );
  quat0 = quat0_cache * Eigen::Quaterniond( Eigen::AngleAxisd( q[ 0 ], axis0_cache ) );
  memcpy( qt + 4, qts_cache.data() + 4, 3 * sizeof( double ) );
  for ( auto k = 1U; k < K; k++ )
  {
    qtm1 = qt;
    qt += QT_SIZE;
    Eigen::Map<const Eigen::Quaterniond> quatk_cache( qts_cache.data() + k * QT_SIZE );
    Eigen::Map<const Vec3d> transk_cache( qts_cache.data() + k * QT_SIZE + 4 );
    Eigen::Map<const Vec3d> axisk_cache( twists_cache.data() + 3 * k );

    Eigen::Map<Eigen::Quaterniond> quatk( qt );
    Eigen::Map<Eigen::Quaterniond> quatkm1( qtm1 );
    Eigen::Map<Vec3d> transk( qt + 4 );
    Eigen::Map<Vec3d> transkm1( qtm1 + 4 );
    quatk  = quatkm1 * quatk_cache * Eigen::Quaterniond( Eigen::AngleAxisd( q[ k ], axisk_cache ) );
    transk = quatkm1 * transk_cache + transkm1;
  }

  qtm1 = qt;
  qt += QT_SIZE;

  Eigen::Map<Eigen::Quaterniond> quat_tool0( qt );
  Eigen::Map<Eigen::Quaterniond> quat_end( qtm1 );
  // std::cout << quat_end_tool0 << std::endl;
  Eigen::Map<Vec3d> trans_tool0( qt + 4 );
  Eigen::Map<Vec3d> trans_end( qtm1 + 4 );

  quat_tool0  = quat_end * quat_end_tool0;
  trans_tool0 = quat_end * t_end_tool0 + trans_end;
}

;

unsigned char ArmR7::ik( const double *tf44_tool0_data, double *q_ik )
{
  constexpr unsigned char UINT8_ONE  = (unsigned char)1;
  constexpr unsigned char UINT8_ZERO = (unsigned char)0;
  unsigned char ik_status            = 0b0;
  const double &q_last               = q_ik[ 6 ];

  double phi4[ 2 ];
  bool phi4_status;

  double phi65[ 4 ];
  bool phi65_status[ 2 ];
  double theta123[ 6 ];
  Eigen::Matrix4d E4;
  Eigen::Vector3d p;

  //  std::cout << q_last << std::endl;

  Mat4d tf44_base_link_end = Eigen::Map<const Mat4d>( tf44_tool0_data ) * tf_tool0_link_end;

  Eigen::Matrix4d tf_link_end_base;
  SE3_inv( tf44_base_link_end, tf_link_end_base );

  Eigen::Matrix4d tf_link7_home_link7_mov = Eigen::Matrix4d ::Identity();
  Eigen::Vector3d axisk_cache( twists_cache.data() + 3 * ( K - 1 ) );

  so3_upexp( q_last * axisk_cache, tf_link7_home_link7_mov.topLeftCorner<3, 3>() );

  Eigen::Matrix4d Ginv = link_end_fk_home * tf_link7_home_link7_mov * tf_link_end_base;

  Eigen::Vector3d GinvPs    = Ginv.topLeftCorner<3, 3>() * q_shoulder + Ginv.topRightCorner<3, 1>();
  const Eigen::Vector3d &w1 = pns_joint[ 0 ].tail<3>();
  const Eigen::Vector3d &w2 = pns_joint[ 1 ].tail<3>();
  const Eigen::Vector3d &w3 = pns_joint[ 2 ].tail<3>();
  const Eigen::Vector3d &r4 = pns_joint[ 3 ].head<3>();
  const Eigen::Vector3d &w4 = pns_joint[ 3 ].tail<3>();
  const Eigen::Vector3d &w5 = pns_joint[ 4 ].tail<3>();
  const Eigen::Vector3d &w6 = pns_joint[ 5 ].tail<3>();
  const double delta        = ( GinvPs - q_wrist ).norm();
  phi4_status               = subproblem_PK3( q_shoulder, q_wrist, r4, w4, &delta, &phi4[ 0 ], &phi4[ 1 ] );

  if ( !phi4_status )
  {
    // std::cout << "# IK FAILS AT !phi4_status" << std::endl;
    return UINT8_ZERO;
  }

  for ( int i_theta4 = 0; i_theta4 < 2; i_theta4++ )
  {
    // E4 = tinycgl::MatExpse3( ::Vec6Tose3( phi4[ i_theta4 ] * poe_twist[ 3 ] ) );
    se3_upexp( phi4[ i_theta4 ] * twists_poe[ 3 ], E4 );
    p = E4.topLeftCorner<3, 3>() * q_shoulder + E4.topRightCorner<3, 1>();

    subproblem_PK2( p, GinvPs, q_wrist, w6, w5, &phi65[ 0 ], &phi65[ 2 ], phi65_status );

    for ( int i_phi65 = 0; i_phi65 < 2; i_phi65++ )
    {
      if ( !phi65_status[ i_phi65 ] )
      {
        // std::cout << " ph 65" << phi4[ 0 ] << " ,  " << phi4[ 1 ] << std::endl;
        // std::cout << "# IK FAILS AT !phi65_status" << std::endl;
        continue;
      }

      Eigen::Matrix3d R_hat = Ginv.topLeftCorner<3, 3>().transpose();
      Eigen::Matrix3d R_tmp;
      so3_upexp( phi65[ i_phi65 * 2 + 0 ] * w6, R_tmp );
      R_hat = R_hat * R_tmp;
      so3_upexp( phi65[ i_phi65 * 2 + 1 ] * w5, R_tmp );
      R_hat = R_hat * R_tmp;
      so3_upexp( phi4[ i_theta4 ] * w4, R_tmp );
      R_hat = R_hat * R_tmp;
      // tinycgl::MatExpso3( tinycgl::Vec3Toso3( phi65[ i_phi65 * 2 + 0 ] * joint_omega[ 5 ] ) ) *
      //     tinycgl::MatExpso3( tinycgl::Vec3Toso3( phi65[ i_phi65 * 2 + 1 ] * joint_omega[ 4 ] ) ) *
      //     tinycgl::MatExpso3( tinycgl::Vec3Toso3( phi4[ i_theta4 ] * joint_omega[ 3 ] ) );

      subproblem_RRR( R_hat, w1, w2, w3, &theta123[ 3 ], &theta123[ 0 ] );
      for ( int i_theta123 = 0; i_theta123 < 2; i_theta123++ )
      {
        uint8_t i_sol = 4 * i_theta4 + 2 * i_phi65 + i_theta123;
        double *q_sol = q_ik + i_sol * 7;
        q_sol[ 0 ]    = theta123[ 3 * i_theta123 + 0 ];
        q_sol[ 1 ]    = theta123[ 3 * i_theta123 + 1 ];
        q_sol[ 2 ]    = theta123[ 3 * i_theta123 + 2 ];
        q_sol[ 3 ]    = -1 * phi4[ i_theta4 ];
        q_sol[ 4 ]    = -1 * phi65[ i_phi65 * 2 + 1 ];
        q_sol[ 5 ]    = -1 * phi65[ i_phi65 * 2 + 0 ];
        q_sol[ 6 ]    = q_last;

        // std::cout << "# [IKPoE] " << i_theta123 << i_theta4 << i_phi65 << "\t = \t";

        for ( int kk = 0; kk < 6; kk++ )
        {
          q_sol[ kk ] = std::atan2( std::sin( q_sol[ kk ] ), std::cos( q_sol[ kk ] ) );

          // printf( "%05.6f\t", q_sol[ kk ] );
        }

        for ( int kk = 0; kk < 6; kk++ )
        {
          if ( q_sol[ kk ] > q_hi[ kk ] ) goto SKIP_SET_IK_VALID;

          if ( q_sol[ kk ] < q_lo[ kk ] ) goto SKIP_SET_IK_VALID;
        }
        // std::cout << std::endl;

        ik_status = ik_status | ( UINT8_ONE << i_sol );
      SKIP_SET_IK_VALID:
        continue;

        // std::cout << "# [IKPoE] " << i_theta123 << i_theta4 << i_phi65 << ": q = "

        //           << theta123[ 3 * i_theta123 + 0 ] << ", " << theta123[ 3 * i_theta123 + 1 ] << ", "
        //           << theta123[ 3 * i_theta123 + 2 ] << ", " << -1 * phi4[ i_theta4 ] << ", "
        //           << -1 * phi65[ i_phi65 * 2 + 1 ] << ", " << -1 * phi65[ i_phi65 * 2 + 0 ] << "\n";
      }
    }
  }

  return UINT8_ZERO;
};
}  // namespace ampl