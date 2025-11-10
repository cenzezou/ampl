

#include "arm_database.hpp"
#include "arms.hpp"
#include "kinematics_common.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ampl/kinematics.hpp>
#include <iostream>
#include <memory>

namespace ampl
{

void ArmR6::initialize_urdf( const double *urdf, const double *joint_limits, const double *xyzrpy_tool0 )
{
  tfs_cache        = urdf2tfs<K, double>( urdf );
  twists_home      = urdf2twists<K, double>( urdf );
  link_end_fk_home = tfs2eehome<K>( tfs_cache.data() );
  pns_joint        = tfs2pn<K, double>( tfs_cache.data(), urdf );
  twists_poe       = find_poetwists<K>( tfs_cache.data(), twists_home.data() );

  // std::cout << "############################## " << std::endl;
  // for ( size_t i = 0; i < K; i++ )
  // {
  //   std::cout << tfs_cache[ i ] << std::endl;
  //   // std::cout << twists_poe[ i ].transpose() << std::endl;
  // }
  // std::cout << "############################## " << std::endl;

  constexpr uint32_t ids_wrist[ 3 ] = { 3, 4, 5 };
  q_wrist                           = find_wrist( pns_joint.data(), ids_wrist );
  qts_cache_f                       = urdf2qts<K, double, float>( urdf );
  qts_cache                         = urdf2qts<K, double, double>( urdf );
  twists_cache                      = urdf2axisarr<K, double>( urdf );

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

  // printf( "# DEBUG initialize_urdf \n" );
  // std::cout << "q_wrist " << q_wrist.transpose() << std::endl;
}
void ArmR6::initialize_preset( const std::string &name )
{
  if ( name == "abb_irb6700_150_320" )
    initialize_urdf( ABB_IRB6700_150_320::urdf, ABB_IRB6700_150_320::bounds, ABB_IRB6700_150_320::tool0 );
  else if ( name == "yaskawa_gp12" )
    initialize_urdf( YASKAWA_GP12::urdf, YASKAWA_GP12::bounds, YASKAWA_GP12::tool0 );
  else if ( name == "elfin_10l" )
    initialize_urdf( ELFIN_10L::urdf, ELFIN_10L::bounds );
};

void ArmR6::set_joint_limits( const double *joint_limits_low, const double *joint_limits_hi )
{
  memcpy( q_lo.data(), joint_limits_low, K * sizeof( double ) );
  memcpy( q_hi.data(), joint_limits_hi, K * sizeof( double ) );
};

void ArmR6::set_link_end_tool0( const double *xyzrpy )
{
  xyzrpy2tf<double>( xyzrpy, xyzrpy + 3, tf_link_end_tool0 );

  quat_end_tool0 = quatd( tf_link_end_tool0.topLeftCorner<3, 3>() );
  t_end_tool0    = tf_link_end_tool0.topRightCorner<3, 1>();

  SE3_inv<double>( tf_link_end_tool0, tf_tool0_link_end );
};

void ArmR6::get_pose_tool0( double *tf44, bool colmajor ){

};
void ArmR6::set_tcp( const double *tf44, bool colmajor )
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

}  // namespace ampl

namespace ampl
{

void ArmR6::fk( const double *q, double *qts_link )
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
};
// void so3_up2( const Eigen::Vector3d &omg, Eigen::Matrix3d &so3 )
// {
//   so3 << -( omg( 1 ) * omg( 1 ) ) - ( omg( 2 ) * omg( 2 ) ), omg( 0 ) * omg( 1 ), omg( 0 ) * omg( 2 ),
//       omg( 0 ) * omg( 1 ), -( omg( 0 ) * omg( 0 ) ) - ( omg( 2 ) * omg( 2 ) ), omg( 1 ) * omg( 2 ), omg( 0 ) * omg( 2
//       ), omg( 1 ) * omg( 2 ), -( omg( 0 ) * omg( 0 ) ) - ( omg( 1 ) * omg( 1 ) );
// }
unsigned char ArmR6::ik( const double *tf44_tool0_data, double *q_ik )
{
  constexpr unsigned char UINT8_ONE = (unsigned char)1;
  Eigen::Matrix4d g;
  Eigen::Vector3d q;
  Eigen::Matrix3d w1Hat2;
  Eigen::Matrix3d R_wrist, R_wrist_single;
  Eigen::Vector3d qnew, pnew, r_wrist_single;
  Eigen::Matrix4d E1Inv, E3;
  Eigen::Vector<double, 6> twist;
  const Eigen::Vector3d &r1 = pns_joint[ 0 ].head<3>();
  const Eigen::Vector3d &r2 = pns_joint[ 1 ].head<3>();
  const Eigen::Vector3d &r3 = pns_joint[ 2 ].head<3>();
  const Eigen::Vector3d &w1 = pns_joint[ 0 ].tail<3>();
  const Eigen::Vector3d &w2 = pns_joint[ 1 ].tail<3>();
  const Eigen::Vector3d &w3 = pns_joint[ 2 ].tail<3>();
  double theta1[ 2 ];
  bool theta1_status[ 2 ];
  double theta2;
  double theta3[ 2 ];
  double theta456A[ 3 ];
  double theta456B[ 3 ];
  double x1, y1, z1, xyz_sqrt, y1_plus_z1, dist_q_r;
  unsigned char ik_status = 0b0;

  Mat4d tf44_link_end = Eigen::Map<const Mat4d>( tf44_tool0_data ) * tf_tool0_link_end;

  SE3_inv( link_end_fk_home, g );
  g = (tf44_link_end)*g;
  q = g.topLeftCorner<3, 3>() * q_wrist + g.topRightCorner<3, 1>();

  // std::cout << "link_end_fk_home" << std::endl;
  // std::cout << link_end_fk_home << std::endl;
  // std::cout << "g" << std::endl;
  // std::cout << g << std::endl;

  {
    so3_up2<double>( w1, w1Hat2 );
    x1          = -w2.dot( w1.cross( q - r1 ) );
    y1          = -w2.dot( w1Hat2 * ( q - r1 ) );
    z1          = -w2.dot( q - q_wrist + w1Hat2 * ( q - r1 ) );
    xyz_sqrt    = x1 * x1 + y1 * y1 - z1 * z1;
    xyz_sqrt    = std::sqrt( xyz_sqrt );
    y1_plus_z1  = y1 + z1;
    theta1[ 0 ] = 2.0 * std::atan2( x1 + xyz_sqrt, y1_plus_z1 );
    theta1[ 1 ] = 2.0 * std::atan2( x1 - xyz_sqrt, y1_plus_z1 );
  }

  for ( uint8_t i_theta1 = 0; i_theta1 < 2; i_theta1++ )
  {
    twist = -1.0 * theta1[ i_theta1 ] * twists_poe[ 0 ];
    se3_upexp( twist, E1Inv );
    // std::cout << theta1[ i_theta1 ] << std::endl;
    // std::cout << E1Inv << std::endl;
    qnew                      = E1Inv.topLeftCorner<3, 3>() * q + E1Inv.topRightCorner<3, 1>();
    dist_q_r                  = ( qnew - r2 ).norm();
    theta1_status[ i_theta1 ] = subproblem_PK3( q_wrist, r2, r3, w3, &dist_q_r, &theta3[ 0 ], &theta3[ 1 ] );
    if ( !theta1_status[ i_theta1 ] ) continue;

    for ( uint8_t i_theta3 = 0; i_theta3 < 2; i_theta3++ )
    {
      twist = theta3[ i_theta3 ] * twists_poe[ 2 ];
      se3_upexp( twist, E3 );
      pnew = E3.topLeftCorner<3, 3>() * q_wrist + E3.topRightCorner<3, 1>();
      subproblem_PK1( pnew, qnew, w2, r2, &theta2 );
      // std::cout << "w2\n" << E3 << std::endl;
      // std::cout << "twists_poe\n" << twists_poe[ 2 ].transpose() << std::endl;
      // std::cout << "theta2" << theta2 << std::endl;

      r_wrist_single = -1.0 * theta3[ i_theta3 ] * w3;
      so3_upexp( r_wrist_single, R_wrist );

      r_wrist_single = -1.0 * theta2 * w2;
      so3_upexp( r_wrist_single, R_wrist_single );
      R_wrist *= R_wrist_single;

      r_wrist_single = -1.0 * theta1[ i_theta1 ] * w1;
      so3_upexp( r_wrist_single, R_wrist_single );
      R_wrist *= R_wrist_single * g.topLeftCorner<3, 3>();

      subproblem_RRR( R_wrist, pns_joint[ 3 ].tail<3>(), pns_joint[ 4 ].tail<3>(), pns_joint[ 5 ].tail<3>(), theta456A,
                      theta456B );

      uint8_t i_sol = 4 * i_theta1 + 2 * i_theta3 + 0;
      double *q_sol = q_ik + (i_sol)*6;
      q_sol[ 0 ]    = theta1[ i_theta1 ];
      q_sol[ 1 ]    = theta2;
      q_sol[ 2 ]    = theta3[ i_theta3 ];
      q_sol[ 3 ]    = theta456A[ 0 ];
      q_sol[ 4 ]    = theta456A[ 1 ];
      q_sol[ 5 ]    = theta456A[ 2 ];

      if ( q_sol[ 0 ] > M_PI ) q_sol[ 0 ] -= M_PI;
      if ( q_sol[ 1 ] > M_PI ) q_sol[ 1 ] -= M_PI;
      if ( q_sol[ 2 ] > M_PI ) q_sol[ 2 ] -= M_PI;
      if ( q_sol[ 0 ] < -M_PI ) q_sol[ 0 ] += M_PI;
      if ( q_sol[ 1 ] < -M_PI ) q_sol[ 1 ] += M_PI;
      if ( q_sol[ 2 ] < -M_PI ) q_sol[ 2 ] += M_PI;

      if ( q_sol[ 3 ] > M_PI ) q_sol[ 3 ] -= M_PI;
      if ( q_sol[ 4 ] > M_PI ) q_sol[ 4 ] -= M_PI;
      if ( q_sol[ 5 ] > M_PI ) q_sol[ 5 ] -= M_PI;
      if ( q_sol[ 3 ] < -M_PI ) q_sol[ 3 ] += M_PI;
      if ( q_sol[ 4 ] < -M_PI ) q_sol[ 4 ] += M_PI;
      if ( q_sol[ 5 ] < -M_PI ) q_sol[ 5 ] += M_PI;

      bool q_sol_012_valid = ( q_sol[ 0 ] >= q_lo[ 0 ] && q_sol[ 0 ] <= q_hi[ 0 ] ) &&  //
                             ( q_sol[ 1 ] >= q_lo[ 1 ] && q_sol[ 1 ] <= q_hi[ 1 ] ) &&  //
                             ( q_sol[ 2 ] >= q_lo[ 2 ] && q_sol[ 2 ] <= q_hi[ 2 ] );

      if (                                                           //
          q_sol_012_valid &&                                         //
          ( q_sol[ 3 ] >= q_lo[ 3 ] && q_sol[ 3 ] <= q_hi[ 3 ] ) &&  //
          ( q_sol[ 4 ] >= q_lo[ 4 ] && q_sol[ 4 ] <= q_hi[ 4 ] ) &&  //
          ( q_sol[ 5 ] >= q_lo[ 5 ] && q_sol[ 5 ] <= q_hi[ 5 ] )     //
      )
      {
        ik_status = ik_status | ( UINT8_ONE << i_sol );
      };

      i_sol += 1;
      q_sol += 6;
      q_sol[ 0 ] = theta1[ i_theta1 ];
      q_sol[ 1 ] = theta2;
      q_sol[ 2 ] = theta3[ i_theta3 ];
      // ik_status  = ik_status | ( UINT8_ONE << i_sol );
      q_sol[ 3 ] = theta456B[ 0 ];
      q_sol[ 4 ] = theta456B[ 1 ];
      q_sol[ 5 ] = theta456B[ 2 ];

      if ( q_sol[ 3 ] > M_PI ) q_sol[ 3 ] -= M_PI;
      if ( q_sol[ 4 ] > M_PI ) q_sol[ 4 ] -= M_PI;
      if ( q_sol[ 5 ] > M_PI ) q_sol[ 5 ] -= M_PI;
      if ( q_sol[ 3 ] < -M_PI ) q_sol[ 3 ] += M_PI;
      if ( q_sol[ 4 ] < -M_PI ) q_sol[ 4 ] += M_PI;
      if ( q_sol[ 5 ] < -M_PI ) q_sol[ 5 ] += M_PI;

      if (  //
            // ( q_sol[ 0 ] >= q_lo[ 0 ] && q_sol[ 0 ] <= q_hi[ 0 ] ) &&  //
            // ( q_sol[ 1 ] >= q_lo[ 1 ] && q_sol[ 1 ] <= q_hi[ 1 ] ) &&  //
            // ( q_sol[ 2 ] >= q_lo[ 2 ] && q_sol[ 2 ] <= q_hi[ 2 ] ) &&  //
          q_sol_012_valid &&                                         //
          ( q_sol[ 3 ] >= q_lo[ 3 ] && q_sol[ 3 ] <= q_hi[ 3 ] ) &&  //
          ( q_sol[ 4 ] >= q_lo[ 4 ] && q_sol[ 4 ] <= q_hi[ 4 ] ) &&  //
          ( q_sol[ 5 ] >= q_lo[ 5 ] && q_sol[ 5 ] <= q_hi[ 5 ] )     //
      )
      {
        ik_status = ik_status | ( UINT8_ONE << i_sol );
      };
    }
  }

  // print_bits( ik_status );

  return ik_status;
}

}  // namespace ampl